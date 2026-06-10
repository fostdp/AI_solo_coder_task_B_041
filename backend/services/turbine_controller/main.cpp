#include <iostream>
#include <memory>
#include <csignal>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <mutex>
#include <cmath>
#include <sstream>
#include <iomanip>

#include "../../include/config.h"
#include "../../include/turbine_mpc_controller.h"
#include "../../include/clickhouse_client.h"
#include "../../../common/include/ipc_queue.h"
#include "../../../common/include/service_base.h"
#include "../../../common/include/metrics.h"

using namespace turbine_monitor;

std::atomic<bool> g_running(true);

void signalHandler(int) {
    std::cout << "\n[turbine_controller] Received shutdown signal" << std::endl;
    g_running = false;
}

class TurbineControllerService : public ServiceBase {
public:
    TurbineControllerService() : ServiceBase("turbine_controller", 3) {}

    bool init(const Config& config) override {
        config_ = config;

        MPCWeights weights{
            config_.mpc.weight_efficiency,
            config_.mpc.weight_cavitation_risk,
            config_.mpc.weight_power_tracking,
            config_.mpc.weight_control_rate
        };

        mpc_ = std::make_unique<ModelPredictiveController>(
            config_.mpc.prediction_horizon,
            config_.mpc.control_horizon,
            weights,
            config_.mpc.sample_time_s);

        mpc_->setMaxIterations(static_cast<int>(config_.mpc.mpc_max_iterations));
        mpc_->setTolerance(config_.mpc.mpc_tolerance);

        MPCConstraints constraints{};
        constraints.guide_vane_min = config_.mpc.guide_vane_min_deg;
        constraints.guide_vane_max = config_.mpc.guide_vane_max_deg;
        constraints.guide_vane_rate_max = config_.mpc.guide_vane_rate_limit_dps;
        constraints.power_min = config_.mpc.power_min_mw;
        constraints.power_max = config_.mpc.power_max_mw;
        constraints.power_rate_max = config_.mpc.power_rate_limit_mwps;
        constraints.cav_risk_max = config_.mpc.cavitation_emergency_threshold;
        mpc_->setConstraints(constraints);

        mpc_->setTarget({500.0f, 0.2f});

        clickhouseClient_ = std::make_unique<ClickHouseClient>(
            config_.clickhouse_host, config_.clickhouse_port,
            config_.clickhouse_user, config_.clickhouse_pass,
            config_.clickhouse_db);

        cavitationQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageCavitation>>(
            IPCChannel::CAVITATION, IPC_DEFAULT_CAPACITY, false);
        if (!cavitationQueue_->open()) {
            std::cerr << "[" << name_ << "] Failed to open CAVITATION IPC (consumer)" << std::endl;
            return false;
        }

        lifeQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageLife>>(
            IPCChannel::LIFE, IPC_DEFAULT_CAPACITY, false);
        if (!lifeQueue_->open()) {
            std::cout << "[" << name_ << "] LIFE channel not available (optional)" << std::endl;
        }

        controlQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageControl>>(
            IPCChannel::CONTROL, IPC_DEFAULT_CAPACITY, true);
        if (!controlQueue_->open()) {
            std::cerr << "[" << name_ << "] Failed to open CONTROL IPC (producer)" << std::endl;
            return false;
        }

        currentStates_.resize(TURBINE_COUNT + 1);
        lastInputs_.resize(TURBINE_COUNT + 1);
        avgCavIntensity_.resize(TURBINE_COUNT + 1, 0.0f);
        lifeDamageCache_.resize((TURBINE_COUNT + 1) * (BLADE_COUNT + 1), 0.0f);

        for (uint8_t tid = 1; tid <= TURBINE_COUNT; ++tid) {
            auto& st = currentStates_[tid];
            st.guide_vane = 50.0f;
            st.power = 500.0f;
            st.head = 120.0f;
            st.flow = 600.0f;
            st.cav_risk = 0.15f;
            lastInputs_[tid] = MPCInput{0.0f, 0.0f};
        }

        initMetrics();
        return true;
    }

    bool start() override {
        if (!clickhouseClient_->connect()) {
            std::cerr << "[" << name_ << "] ClickHouse connect failed, running without DB" << std::endl;
        }
        clickhouseClient_->startBatchWriter(BATCH_WRITE_SIZE, 1000);

        controlThread_ = std::thread([this]() { this->controlLoop(); });
        statsThread_ = std::thread([this]() { this->statsLoop(); });

        std::cout << "========================================" << std::endl;
        std::cout << "[turbine_controller] Started successfully" << std::endl;
        std::cout << "  IPC CAVITATION -> consumer" << std::endl;
        std::cout << "  IPC LIFE -> optional consumer" << std::endl;
        std::cout << "  IPC CONTROL -> producer" << std::endl;
        std::cout << "  Prediction horizon: Np=" << config_.mpc.prediction_horizon << std::endl;
        std::cout << "  Control horizon: Nc=" << config_.mpc.control_horizon << std::endl;
        std::cout << "  Sample time: " << config_.mpc.sample_time_s << "s" << std::endl;
        std::cout << "  Max iterations: " << config_.mpc.mpc_max_iterations << std::endl;
        std::cout << "========================================" << std::endl;

        return ServiceBase::start();
    }

    void stop() override {
        ServiceBase::stop();
        if (controlThread_.joinable()) controlThread_.join();
        if (statsThread_.joinable()) statsThread_.join();
        clickhouseClient_->stopBatchWriter();
        clickhouseClient_->disconnect();
        cavitationQueue_->close();
        lifeQueue_->close();
        controlQueue_->close();
    }

    void join() override {
        while (running_ && g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        stop();
    }

private:
    void initMetrics() {
        auto& reg = MetricsRegistry::instance();

        auto& hist_fam = prometheus::BuildHistogram()
            .Name("mpc_solve_time_seconds")
            .Help("MPC solve time histogram")
            .Register(*reg.registry_);
        mpc_solve_time_histogram_ = &hist_fam.Add({}, prometheus::Histogram::BucketBoundaries{
            0.0001, 0.0005, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1});

        auto& ctrl_fam = prometheus::BuildCounter()
            .Name("control_action_total")
            .Help("Control actions by mode")
            .Register(*reg.registry_);
        control_action_manual_ = &ctrl_fam.Add({{"mode", "manual"}});
        control_action_eff_ = &ctrl_fam.Add({{"mode", "efficiency_only"}});
        control_action_cav_ = &ctrl_fam.Add({{"mode", "cavitation_safe"}});
        control_action_mpc_ = &ctrl_fam.Add({{"mode", "mpc_optimal"}});

        auto& cav_gauge_fam = prometheus::BuildGauge()
            .Name("mpc_cavitation_risk_gauge")
            .Help("MPC predicted cavitation risk per turbine")
            .Register(*reg.registry_);
        cav_risk_gauge_ = &cav_gauge_fam;

        auto& eff_gauge_fam = prometheus::BuildGauge()
            .Name("mpc_efficiency_pred_gauge")
            .Help("MPC predicted efficiency per turbine")
            .Register(*reg.registry_);
        efficiency_pred_gauge_ = &eff_gauge_fam;
    }

    void controlLoop() {
        const int loop_interval_ms = 100;

        while (g_running && running_) {
            auto loop_start = std::chrono::steady_clock::now();

            processCavitationMessages();
            processLifeMessages();
            runMPCForAllTurbines();

            auto loop_end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                loop_end - loop_start).count();

            int sleep_ms = loop_interval_ms - static_cast<int>(elapsed);
            if (sleep_ms > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            }
        }
    }

    void processCavitationMessages() {
        IPCMessageCavitation cavMsg{};
        size_t processed = 0;
        const size_t max_batch = 500;

        while (processed < max_batch && cavitationQueue_->pop(cavMsg) && g_running) {
            uint8_t tid = cavMsg.turbine_id;
            if (tid > 0 && tid <= TURBINE_COUNT) {
                uint8_t bid = cavMsg.blade_id;
                if (bid > 0 && bid <= BLADE_COUNT) {
                    cavitationCache_[(tid - 1) * BLADE_COUNT + (bid - 1)] = cavMsg.cavitation_intensity;
                }

                float& avg = avgCavIntensity_[tid];
                avg = 0.9f * avg + 0.1f * cavMsg.cavitation_intensity;
                currentStates_[tid].cav_risk = avg;
            }
            processed++;
        }

        if (processed > 0) {
            std::lock_guard<std::mutex> lk(statsMutex_);
            stats_.cavitationConsumed += processed;
        }
    }

    void processLifeMessages() {
        IPCMessageLife lifeMsg{};
        size_t processed = 0;
        const size_t max_batch = 200;

        while (processed < max_batch && lifeQueue_->pop(lifeMsg) && g_running) {
            uint8_t tid = lifeMsg.turbine_id;
            uint8_t bid = lifeMsg.blade_id;
            if (tid > 0 && tid <= TURBINE_COUNT && bid > 0 && bid <= BLADE_COUNT) {
                lifeDamageCache_[(tid - 1) * BLADE_COUNT + (bid - 1)] = lifeMsg.cumulative_damage;
            }
            processed++;
        }

        if (processed > 0) {
            std::lock_guard<std::mutex> lk(statsMutex_);
            stats_.lifeConsumed += processed;
        }
    }

    void runMPCForAllTurbines() {
        for (uint8_t tid = 1; tid <= TURBINE_COUNT; ++tid) {
            auto& state = currentStates_[tid];

            updateStateFromEnv(state, tid);

            auto t0 = std::chrono::steady_clock::now();

            TurbineControlCommand cmd = mpc_->solve(state, lastInputs_[tid], tid);

            auto t1 = std::chrono::steady_clock::now();
            double solve_time = std::chrono::duration<double>(t1 - t0).count();
            mpc_solve_time_histogram_->Observe(solve_time);

            control_action_mpc_->Increment();

            efficiency_pred_gauge_->Add({
                {"turbine", std::to_string(tid)}
            }).Set(static_cast<double>(cmd.predicted_efficiency));

            cav_risk_gauge_->Add({
                {"turbine", std::to_string(tid)}
            }).Set(static_cast<double>(cmd.predicted_cavitation_risk));

            IPCMessageControl ctrlMsg{};
            ctrlMsg.timestamp = cmd.timestamp;
            ctrlMsg.turbine_id = cmd.turbine_id;
            ctrlMsg.control_mode = static_cast<uint8_t>(cmd.control_mode);
            ctrlMsg.cavitation_avoidance_enabled = cmd.cavitation_avoidance_enabled ? 1 : 0;
            ctrlMsg.guide_vane_opening_deg = cmd.guide_vane_opening_deg;
            ctrlMsg.target_power_mw = cmd.target_power_mw;
            ctrlMsg.current_head_m = cmd.current_head_m;
            ctrlMsg.current_flow_m3s = cmd.current_flow_m3s;
            ctrlMsg.predicted_efficiency = cmd.predicted_efficiency;
            ctrlMsg.predicted_cavitation_risk = cmd.predicted_cavitation_risk;
            ctrlMsg.mpc_cost_value = cmd.mpc_cost_value;

            size_t cs_count = std::min(cmd.control_signals.size(), size_t(8));
            for (size_t i = 0; i < cs_count; ++i) {
                ctrlMsg.control_signal[i] = cmd.control_signals[i];
            }

            size_t hs_count = std::min(cmd.horizon_states.size(), size_t(32));
            for (size_t i = 0; i < hs_count; ++i) {
                ctrlMsg.horizon_states[i] = cmd.horizon_states[i];
            }

            size_t desc_len = std::min(cmd.control_action_desc.size(), size_t(127));
            for (size_t i = 0; i < desc_len; ++i) {
                ctrlMsg.control_action_desc[i] = cmd.control_action_desc[i];
            }
            ctrlMsg.control_action_desc[desc_len] = '\0';

            if (!controlQueue_->push(ctrlMsg)) {
                std::lock_guard<std::mutex> lk(statsMutex_);
                stats_.controlDropped++;
            } else {
                std::lock_guard<std::mutex> lk(statsMutex_);
                stats_.controlProduced++;
            }

            clickhouseClient_->insertAlarmLog(convertToAlarmLog(cmd));

            state.guide_vane = cmd.guide_vane_opening_deg;
            state.power = cmd.target_power_mw;
            lastInputs_[tid] = MPCInput{
                cmd.control_signals.size() > 0 ? cmd.control_signals[0] : 0.0f,
                cmd.control_signals.size() > 1 ? cmd.control_signals[1] : 0.0f
            };
        }
    }

    void updateStateFromEnv(MPCState& state, uint8_t tid) {
        (void)tid;
        float noise = 0.0f;

        state.head = 120.0f + noise * 5.0f;
        float flow_base = 620.0f;
        float gv_factor = (state.guide_vane - 50.0f) / 35.0f;
        state.flow = flow_base + gv_factor * 180.0f;

        if (std::abs(state.flow - 620.0f) < 50.0f) {
            state.head = 120.0f;
        } else if (state.flow > 620.0f) {
            state.head -= (state.flow - 620.0f) * 0.03f;
        } else {
            state.head += (620.0f - state.flow) * 0.02f;
        }

        float eff = mpc_->efficiencyModel().getEfficiency(
            state.head, state.flow, state.guide_vane);
        state.power = 9.81f * state.flow * state.head * eff / 1000.0f;
        state.power = std::max(config_.mpc.power_min_mw,
                         std::min(config_.mpc.power_max_mw, state.power));
    }

    AlarmLog convertToAlarmLog(const TurbineControlCommand& cmd) {
        AlarmLog log{};
        log.timestamp = cmd.timestamp;

        std::ostringstream oss;
        oss << "CTRL-" << cmd.turbine_id << "-" << cmd.timestamp;
        log.alarm_id = oss.str();

        log.turbine_id = cmd.turbine_id;
        log.blade_id = 0;

        if (cmd.predicted_cavitation_risk > config_.mpc.cavitation_emergency_threshold) {
            log.alarm_type = AlarmType::CAVITATION_EXCEED;
            log.alarm_level = AlarmLevel::EMERGENCY;
        } else if (cmd.predicted_cavitation_risk > config_.mpc.cavitation_safe_threshold) {
            log.alarm_type = AlarmType::CAVITATION_EXCEED;
            log.alarm_level = AlarmLevel::WARNING;
        } else {
            log.alarm_type = AlarmType::CAVITATION_EXCEED;
            log.alarm_level = static_cast<AlarmLevel>(0);
        }

        log.alarm_message = cmd.control_action_desc;
        log.threshold_value = config_.mpc.cavitation_safe_threshold;
        log.actual_value = cmd.predicted_cavitation_risk;
        log.iec61850_pushed = false;
        log.acknowledged = false;

        std::ostringstream sugg;
        sugg << std::fixed << std::setprecision(1);
        sugg << "Eff=" << (cmd.predicted_efficiency * 100.0f) << "%";
        sugg << ", GV=" << cmd.guide_vane_opening_deg << "deg";
        sugg << ", P=" << cmd.target_power_mw << "MW";
        log.maintenance_suggestion = sugg.str();

        log.acknowledged_at = 0;
        log.acknowledged_by = "";

        return log;
    }

    void statsLoop() {
        uint64_t lastPrint = 0;
        while (g_running && running_) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            uint64_t now = currentTimestampMs();
            if (now - lastPrint < 5000) continue;
            lastPrint = now;

            auto cavStats = cavitationQueue_->getStats();
            auto ctrlStats = controlQueue_->getStats();

            std::lock_guard<std::mutex> lk(statsMutex_);
            std::cout << "\r[turbine_controller] "
                      << "cav_in=" << stats_.cavitationConsumed
                      << " | ctrl_out=" << stats_.controlProduced
                      << " | drop=" << stats_.controlDropped
                      << " | cav_q=" << cavStats.currentSize
                      << " | ctrl_q=" << ctrlStats.currentSize
                      << std::flush;
        }
    }

    Config config_;
    std::unique_ptr<ModelPredictiveController> mpc_;
    std::unique_ptr<ClickHouseClient> clickhouseClient_;

    std::unique_ptr<SharedMemorySPSC<IPCMessageCavitation>> cavitationQueue_;
    std::unique_ptr<SharedMemorySPSC<IPCMessageLife>> lifeQueue_;
    std::unique_ptr<SharedMemorySPSC<IPCMessageControl>> controlQueue_;

    std::vector<MPCState> currentStates_;
    std::vector<MPCInput> lastInputs_;
    std::vector<float> avgCavIntensity_;
    std::vector<float> cavitationCache_ = std::vector<float>((TURBINE_COUNT + 1) * (BLADE_COUNT + 1), 0.0f);
    std::vector<float> lifeDamageCache_;

    std::thread controlThread_;
    std::thread statsThread_;

    prometheus::Histogram* mpc_solve_time_histogram_;
    prometheus::Counter* control_action_manual_;
    prometheus::Counter* control_action_eff_;
    prometheus::Counter* control_action_cav_;
    prometheus::Counter* control_action_mpc_;
    prometheus::Family<prometheus::Gauge>* cav_risk_gauge_;
    prometheus::Family<prometheus::Gauge>* efficiency_pred_gauge_;

    struct ServiceStats {
        uint64_t cavitationConsumed = 0;
        uint64_t lifeConsumed = 0;
        uint64_t controlProduced = 0;
        uint64_t controlDropped = 0;
    };
    ServiceStats stats_;
    std::mutex statsMutex_;
};

int main(int argc, char* argv[]) {
    std::string configFile = "../../../config/config.json";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            configFile = argv[++i];
        }
    }

    if (!loadConfig(configFile)) {
        std::cerr << "[turbine_controller] Warning: Failed to load config, using defaults" << std::endl;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const Config& cfg = getConfig();

    try {
        MetricsRegistry::instance().init("0.0.0.0:9103");
    } catch (const std::exception& e) {
        std::cerr << "[turbine_controller] Metrics init failed: " << e.what() << std::endl;
    }

    auto service = std::make_unique<TurbineControllerService>();
    if (!service->init(cfg)) {
        std::cerr << "[turbine_controller] Init failed" << std::endl;
        return 1;
    }

    if (!service->start()) {
        std::cerr << "[turbine_controller] Start failed" << std::endl;
        return 1;
    }

    service->join();
    std::cout << std::endl << "[turbine_controller] Shutdown complete" << std::endl;
    return 0;
}
