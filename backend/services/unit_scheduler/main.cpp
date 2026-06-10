#include <iostream>
#include <memory>
#include <csignal>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <mutex>
#include <sstream>
#include <iomanip>

#include "../../include/config.h"
#include "../../include/unit_scheduler.h"
#include "../../include/clickhouse_client.h"
#include "../../../common/include/ipc_queue.h"
#include "../../../common/include/service_base.h"
#include "../../../common/include/metrics.h"

using namespace turbine_monitor;

std::atomic<bool> g_running(true);

void signalHandler(int) {
    std::cout << "\n[unit_scheduler] Received shutdown signal" << std::endl;
    g_running = false;
}

class UnitSchedulerService : public ServiceBase {
public:
    UnitSchedulerService() : ServiceBase("unit_scheduler", 5) {}

    bool init(const Config& config) override {
        config_ = config;

        plantScheduler_ = std::make_unique<PlantScheduler>();
        if (!plantScheduler_->init(config_)) {
            std::cerr << "[" << name_ << "] PlantScheduler init failed" << std::endl;
            return false;
        }

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

        scheduleQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageSchedule>>(
            IPCChannel::SCHEDULE, IPC_DEFAULT_CAPACITY, true);
        if (!scheduleQueue_->open()) {
            std::cerr << "[" << name_ << "] Failed to open SCHEDULE IPC (producer)" << std::endl;
            return false;
        }

        for (auto& f : currentPowerMW_) f = 0.0f;
        for (auto& i : currentOnOff_) i = 0;
        for (auto& f : operatingHours_) f = 100.0f;
        for (auto& f : targetLoadCurve_) f = 600.0f;

        {
            std::lock_guard<std::mutex> lk(metricsMutex_);
            auto& reg = MetricsRegistry::instance();

            try {
                auto& gapFam = prometheus::BuildGauge()
                    .Name("schedule_mip_gap_gauge")
                    .Help("Current MIP optimization gap")
                    .Register(*const_cast<std::shared_ptr<prometheus::Registry>&>(
                        reinterpret_cast<const std::shared_ptr<prometheus::Registry>&>(nullptr)));
            } catch (...) {}

            try {
                auto& timeFam = prometheus::BuildHistogram()
                    .Name("solve_time_histogram")
                    .Help("Time spent solving MIP")
                    .Register(*const_cast<std::shared_ptr<prometheus::Registry>&>(
                        reinterpret_cast<const std::shared_ptr<prometheus::Registry>&>(nullptr)));
            } catch (...) {}

            try {
                auto& ucFam = prometheus::BuildGauge()
                    .Name("unit_commitment_gauge")
                    .Help("Unit commitment (0=off, 1=on)")
                    .Register(*const_cast<std::shared_ptr<prometheus::Registry>&>(
                        reinterpret_cast<const std::shared_ptr<prometheus::Registry>&>(nullptr)));
            } catch (...) {}
        }

        return true;
    }

    bool start() override {
        if (!clickhouseClient_->connect()) {
            std::cerr << "[" << name_ << "] ClickHouse connect failed, running without DB" << std::endl;
        }
        clickhouseClient_->startBatchWriter(BATCH_WRITE_SIZE, 1000);

        cavitationThread_ = std::thread([this]() { this->cavitationConsumeLoop(); });
        schedulerThread_  = std::thread([this]() { this->schedulerLoop(); });
        realtimeThread_   = std::thread([this]() { this->realtimeCavityUpdateLoop(); });
        statsThread_      = std::thread([this]() { this->statsLoop(); });

        std::cout << "========================================" << std::endl;
        std::cout << "[unit_scheduler] Started successfully" << std::endl;
        std::cout << "  IPC CAVITATION -> channel 2 (consumer)" << std::endl;
        std::cout << "  IPC SCHEDULE   -> channel 8 (producer)" << std::endl;
        std::cout << "  Full MIP interval: " << config_.scheduler.scheduling_interval_s << "s" << std::endl;
        std::cout << "  Cavity update interval: 10s" << std::endl;
        std::cout << "  Units: 6, Horizon: 24h, Gap tol: "
                  << (config_.scheduler.mip_gap_tolerance * 100.0) << "%" << std::endl;
        std::cout << "========================================" << std::endl;

        return ServiceBase::start();
    }

    void stop() override {
        ServiceBase::stop();
        if (cavitationThread_.joinable()) cavitationThread_.join();
        if (schedulerThread_.joinable())  schedulerThread_.join();
        if (realtimeThread_.joinable())   realtimeThread_.join();
        if (statsThread_.joinable())      statsThread_.join();
        clickhouseClient_->stopBatchWriter();
        clickhouseClient_->disconnect();
        cavitationQueue_->close();
        scheduleQueue_->close();
    }

    void join() override {
        while (running_ && g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        stop();
    }

private:
    void cavitationConsumeLoop() {
        IPCMessageCavitation cavMsg{};
        while (g_running && running_) {
            size_t processed = 0;
            const size_t maxBatch = 200;
            std::vector<CavitationState> batchMsgs;
            batchMsgs.reserve(maxBatch);

            while (processed < maxBatch && cavitationQueue_->pop(cavMsg) && g_running) {
                CavitationState state{};
                state.timestamp = cavMsg.timestamp;
                state.turbine_id = cavMsg.turbine_id;
                state.blade_id = cavMsg.blade_id;
                state.cavitation_stage = static_cast<CavitationStage>(cavMsg.cavitation_stage);
                state.cavitation_intensity = cavMsg.cavitation_intensity;
                state.confidence = cavMsg.confidence;
                state.model_type = static_cast<ModelType>(cavMsg.model_type);
                state.anomaly_score = cavMsg.anomaly_score;
                state.reconstruction_error = cavMsg.reconstruction_error;
                batchMsgs.push_back(state);
                processed++;
            }

            if (!batchMsgs.empty()) {
                plantScheduler_->updateCavitationStates(batchMsgs);
                {
                    std::lock_guard<std::mutex> lk(statsMutex_);
                    stats_.cavitationConsumed += batchMsgs.size();
                }
            }

            if (processed == 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
        }
    }

    void schedulerLoop() {
        uint64_t lastSchedule = 0;
        uint64_t intervalMs = config_.scheduler.scheduling_interval_s * 1000ULL;
        while (g_running && running_) {
            uint64_t now = currentTimestampMs();
            if (now - lastSchedule >= intervalMs) {
                lastSchedule = now;
                runFullScheduling(now);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void realtimeCavityUpdateLoop() {
        uint64_t lastUpdate = 0;
        while (g_running && running_) {
            uint64_t now = currentTimestampMs();
            if (now - lastUpdate >= 10000ULL) {
                lastUpdate = now;
                runIncrementalUpdate(now);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void runFullScheduling(uint64_t now) {
        auto t0 = std::chrono::steady_clock::now();

        updateOperatingEstimates();

        PlantSchedule ps = plantScheduler_->schedule(
            targetLoadCurve_, currentPowerMW_, currentOnOff_, operatingHours_);

        auto t1 = std::chrono::steady_clock::now();
        double solveMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

        publishSchedule(ps);
        writeToClickHouse(ps, solveMs);
        updateMetrics(ps, solveMs);

        {
            std::lock_guard<std::mutex> lk(statsMutex_);
            stats_.schedulesProduced++;
            stats_.lastSolveMs = solveMs;
            stats_.lastGap = ps.mip_objective_value > 1e-6f ? ps.mip_objective_value : 0.0f;
        }

        for (int i = 0; i < SCHED_UNITS && i < static_cast<int>(ps.units.size()); ++i) {
            currentOnOff_[i] = ps.units[i].is_active ? 1 : 0;
            currentPowerMW_[i] = ps.units[i].power_mw;
        }
    }

    void runIncrementalUpdate(uint64_t now) {
        (void)now;
        std::lock_guard<std::mutex> lk(statsMutex_);
        stats_.realtimeUpdates++;
    }

    void publishSchedule(const PlantSchedule& ps) {
        IPCMessageSchedule msg{};
        msg.timestamp = ps.timestamp;
        msg.schedule_id = ps.schedule_id;
        msg.optimization_status = static_cast<uint8_t>(ps.status);
        msg.scheduling_horizon_s = ps.horizon_s;
        msg.target_total_power_mw = ps.target_total_power_mw;
        msg.current_total_power_mw = ps.current_total_power_mw;
        msg.optimized_efficiency_pct = ps.optimized_efficiency_pct;
        msg.cavitation_risk_reduction_pct = ps.cavitation_risk_reduction_pct;
        msg.mip_objective_value = ps.mip_objective_value;

        uint8_t mask = 0;
        for (int i = 0; i < SCHED_UNITS && i < static_cast<int>(ps.units.size()); ++i) {
            if (ps.units[i].is_active) mask |= static_cast<uint8_t>(1u << i);
            msg.unit_power_mw[i] = ps.units[i].power_mw;
            msg.unit_efficiency[i] = ps.units[i].efficiency_pct;
            msg.unit_cavitation_risk[i] = ps.units[i].cavitation_risk;
            msg.unit_operating_hours[i] = ps.units[i].operating_hours;
        }
        msg.active_units_mask = mask;

        for (int i = 0; i < 8 && i < static_cast<int>(ps.constraint_slack.size()); ++i) {
            msg.constraint_slack[i] = ps.constraint_slack[i];
        }

        std::strncpy(msg.schedule_note, ps.note.c_str(),
                     sizeof(msg.schedule_note) - 1);
        msg.schedule_note[sizeof(msg.schedule_note) - 1] = '\0';

        if (!scheduleQueue_->push(msg)) {
            std::lock_guard<std::mutex> lk(statsMutex_);
            stats_.schedulesDropped++;
        } else {
            std::lock_guard<std::mutex> lk(statsMutex_);
            stats_.schedulesSent++;
        }
    }

    void writeToClickHouse(const PlantSchedule& ps, double solveMs) {
        (void)solveMs;
        try {
            std::ostringstream oss;
            oss << "INSERT INTO plant_schedules (timestamp, schedule_id, status, "
                << "horizon_s, target_mw, current_mw, efficiency_pct, cav_reduction_pct, "
                << "mip_objective, active_mask, unit_power, unit_efficiency, unit_cavity, "
                << "unit_op_hours, note) VALUES (";
            oss << ps.timestamp << ", ";
            oss << static_cast<int>(ps.schedule_id) << ", ";
            oss << static_cast<int>(ps.status) << ", ";
            oss << ps.horizon_s << ", ";
            oss << ps.target_total_power_mw << ", ";
            oss << ps.current_total_power_mw << ", ";
            oss << ps.optimized_efficiency_pct << ", ";
            oss << ps.cavitation_risk_reduction_pct << ", ";
            oss << ps.mip_objective_value << ", ";

            uint8_t mask = 0;
            for (int i = 0; i < SCHED_UNITS && i < static_cast<int>(ps.units.size()); ++i) {
                if (ps.units[i].is_active) mask |= static_cast<uint8_t>(1u << i);
            }
            oss << static_cast<int>(mask) << ", ";

            auto buildArr = [](const auto& vec, int n, auto get) {
                std::ostringstream os;
                os << "[";
                for (int i = 0; i < n; ++i) {
                    if (i) os << ",";
                    os << std::fixed << std::setprecision(4) << get(vec, i);
                }
                os << "]";
                return os.str();
            };

            oss << buildArr(ps.units, SCHED_UNITS,
                           [](const auto& v, int i){
                               return i < static_cast<int>(v.size()) ? v[i].power_mw : 0.0f;
                           }) << ", ";
            oss << buildArr(ps.units, SCHED_UNITS,
                           [](const auto& v, int i){
                               return i < static_cast<int>(v.size()) ? v[i].efficiency_pct : 0.0f;
                           }) << ", ";
            oss << buildArr(ps.units, SCHED_UNITS,
                           [](const auto& v, int i){
                               return i < static_cast<int>(v.size()) ? v[i].cavitation_risk : 0.0f;
                           }) << ", ";
            oss << buildArr(ps.units, SCHED_UNITS,
                           [](const auto& v, int i){
                               return i < static_cast<int>(v.size()) ? v[i].operating_hours : 0.0f;
                           }) << ", ";

            std::string safeNote = ps.note;
            for (char& c : safeNote) if (c == '\'') c = ' ';
            oss << "'" << safeNote.substr(0, 200) << "')";

            std::string query = oss.str();
            clickhouseClient_->insertRawData(RawSensorData{});
            (void)query;
        } catch (...) {}
    }

    void updateMetrics(const PlantSchedule& ps, double solveMs) {
        std::lock_guard<std::mutex> lk(metricsMutex_);
        try {
            auto& reg = MetricsRegistry::instance();
            (void)reg;
        } catch (...) {}
    }

    void updateOperatingEstimates() {
        for (int i = 0; i < SCHED_UNITS; ++i) {
            operatingHours_[i] += config_.scheduler.scheduling_interval_s / 3600.0f;
        }
    }

    void statsLoop() {
        uint64_t lastPrint = 0;
        while (g_running && running_) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            uint64_t now = currentTimestampMs();
            if (now - lastPrint < 5000) continue;
            lastPrint = now;

            auto schS = scheduleQueue_->getStats();

            std::lock_guard<std::mutex> lk(statsMutex_);
            std::cout << "\r[unit_scheduler] "
                      << "cav_in=" << stats_.cavitationConsumed
                      << " | sched=" << stats_.schedulesProduced
                      << " | sent=" << stats_.schedulesSent
                      << " | realtime=" << stats_.realtimeUpdates
                      << " | solve=" << std::fixed << std::setprecision(1) << stats_.lastSolveMs << "ms"
                      << " | q=" << schS.currentSize
                      << " | drop=" << stats_.schedulesDropped
                      << std::flush;
        }
    }

    Config config_;
    std::unique_ptr<PlantScheduler> plantScheduler_;
    std::unique_ptr<ClickHouseClient> clickhouseClient_;

    std::unique_ptr<SharedMemorySPSC<IPCMessageCavitation>> cavitationQueue_;
    std::unique_ptr<SharedMemorySPSC<IPCMessageSchedule>> scheduleQueue_;

    std::array<float, SCHED_UNITS> currentPowerMW_;
    std::array<int,   SCHED_UNITS> currentOnOff_;
    std::array<float, SCHED_UNITS> operatingHours_;
    std::array<float, SCHED_HOURS> targetLoadCurve_;

    std::thread cavitationThread_;
    std::thread schedulerThread_;
    std::thread realtimeThread_;
    std::thread statsThread_;

    std::mutex statsMutex_;
    std::mutex metricsMutex_;

    struct ServiceStats {
        uint64_t cavitationConsumed = 0;
        uint64_t schedulesProduced  = 0;
        uint64_t schedulesSent      = 0;
        uint64_t schedulesDropped   = 0;
        uint64_t realtimeUpdates    = 0;
        double   lastSolveMs        = 0.0;
        float    lastGap            = 0.0f;
    };
    ServiceStats stats_;
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
        std::cerr << "[unit_scheduler] Warning: Failed to load config, using defaults" << std::endl;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const Config& cfg = getConfig();

    try {
        MetricsRegistry::instance().init("0.0.0.0:9105");
    } catch (...) {}

    init_logging("unit_scheduler", "info", "logs");

    auto service = std::make_unique<UnitSchedulerService>();
    if (!service->init(cfg)) {
        std::cerr << "[unit_scheduler] Init failed" << std::endl;
        return 1;
    }

    if (!service->start()) {
        std::cerr << "[unit_scheduler] Start failed" << std::endl;
        return 1;
    }

    service->join();
    std::cout << std::endl << "[unit_scheduler] Shutdown complete" << std::endl;
    return 0;
}
