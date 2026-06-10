#include <iostream>
#include <memory>
#include <csignal>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <mutex>
#include <unordered_map>

#include "../../include/config.h"
#include "../../include/data_structures.h"
#include "../../include/api_server.h"
#include "../../../common/include/ipc_queue.h"
#include "../../../common/include/service_base.h"

using namespace turbine_monitor;

std::atomic<bool> g_running(true);

void signalHandler(int) {
    std::cout << "\n[api_gateway] Received shutdown signal" << std::endl;
    g_running = false;
}

class GatewayDataProvider : public DataProvider {
public:
    bool init() {
        cavitationQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageCavitation>>(
            IPCChannel::CAVITATION, IPC_DEFAULT_CAPACITY, false);
        lifeQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageLife>>(
            IPCChannel::LIFE, IPC_DEFAULT_CAPACITY, false);
        alarmQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageAlarm>>(
            IPCChannel::ALARM, IPC_DEFAULT_CAPACITY, false);
        controlQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageControl>>(
            IPCChannel::CONTROL, IPC_DEFAULT_CAPACITY, false);
        robotQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageRobotPlan>>(
            IPCChannel::ROBOT_PLAN, IPC_DEFAULT_CAPACITY, false);
        scheduleQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageSchedule>>(
            IPCChannel::SCHEDULE, IPC_DEFAULT_CAPACITY, false);
        diagnosisQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageDiagnosis>>(
            IPCChannel::DIAGNOSIS, IPC_DEFAULT_CAPACITY, false);

        if (!cavitationQueue_->open() || !lifeQueue_->open() || !alarmQueue_->open() ||
            !controlQueue_->open() || !robotQueue_->open() ||
            !scheduleQueue_->open() || !diagnosisQueue_->open()) {
            std::cerr << "[GatewayDataProvider] Failed to open IPC queues" << std::endl;
            return false;
        }

        turbineConfigs_ = buildTurbineConfigs();
        initDefaultCaches();
        running_ = true;
        pollThread_ = std::thread([this]() { this->pollIPC(); });
        return true;
    }

    void stop() {
        running_ = false;
        if (pollThread_.joinable()) pollThread_.join();
        if (cavitationQueue_) cavitationQueue_->close();
        if (lifeQueue_) lifeQueue_->close();
        if (alarmQueue_) alarmQueue_->close();
        if (controlQueue_) controlQueue_->close();
        if (robotQueue_) robotQueue_->close();
        if (scheduleQueue_) scheduleQueue_->close();
        if (diagnosisQueue_) diagnosisQueue_->close();
    }

    // ==================== Feature 1: MPC Control ====================
    std::vector<TurbineControlStatus> getControlStatus() override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        return controlCache_;
    }

    bool setControlCommand(uint8_t turbineId, ControlMode mode,
                            float guideVaneDeg, float targetPowerMw,
                            bool cavAvoidanceEnable,
                            const std::vector<float>&) override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        for (auto& c : controlCache_) {
            if (c.turbine_id == turbineId) {
                c.mode = mode;
                c.guide_vane_opening_deg = guideVaneDeg;
                c.target_power_mw = targetPowerMw;
                c.cavitation_avoidance_enabled = cavAvoidanceEnable;
                c.timestamp = currentTimestampMs();
                return true;
            }
        }
        return false;
    }

    // ==================== Feature 2: Robot Repair ====================
    std::vector<RobotRepairTask> getRobotTasks() override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        return robotCache_;
    }

    bool triggerRobotPlan(uint8_t turbineId, RepairMode mode,
                           const std::vector<uint8_t>& targetBlades) override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        RobotRepairTask t{};
        t.timestamp = currentTimestampMs();
        t.turbine_id = turbineId;
        t.robot_status = RobotStatus::PLANNING;
        t.repair_mode = mode;
        t.target_blade_ids = targetBlades;
        t.estimated_duration_ms = targetBlades.size() * 600000ULL;
        robotCache_.push_back(t);
        if (robotCache_.size() > 100) robotCache_.erase(robotCache_.begin());
        return true;
    }

    bool cancelRobotTask(const std::string& taskId) override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        for (auto& t : robotCache_) {
            if (std::to_string(t.timestamp) == taskId) {
                t.robot_status = RobotStatus::IDLE;
                return true;
            }
        }
        return false;
    }

    RobotRepairTask getRobotTaskStatus(const std::string& taskId) override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        for (const auto& t : robotCache_) {
            if (std::to_string(t.timestamp) == taskId) return t;
        }
        return RobotRepairTask{};
    }

    // ==================== Feature 3: Unit Scheduler ====================
    PlantSchedule getCurrentSchedule() override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        return scheduleCache_;
    }

    bool runOptimization(float targetTotalPowerMw, uint32_t horizonHours) override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        scheduleCache_.target_total_power_mw = targetTotalPowerMw;
        scheduleCache_.horizon_s = horizonHours * 3600ULL;
        scheduleCache_.status = ScheduleStatus::OPTIMIZING;
        scheduleCache_.timestamp = currentTimestampMs();
        return true;
    }

    bool executeSchedule(uint8_t scheduleId) override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        scheduleCache_.schedule_id = scheduleId;
        scheduleCache_.note = "Schedule " + std::to_string(scheduleId) + " executed";
        return true;
    }

    // ==================== Feature 4: Acoustic Diagnosis ====================
    std::vector<AcousticPattern> getAcousticPatterns() override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        return patternCache_;
    }

    std::vector<DiagnosisResult> getDiagnosisResults(uint32_t limit = 50) override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        std::vector<DiagnosisResult> result;
        size_t start = diagnosisCache_.size() > limit ? diagnosisCache_.size() - limit : 0;
        for (size_t i = start; i < diagnosisCache_.size(); ++i) {
            result.push_back(diagnosisCache_[i]);
        }
        return result;
    }

    bool labelPattern(const std::string& patternId, CavitationType type,
                       const std::string& expertNote, bool verified) override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        for (auto& p : patternCache_) {
            if (std::to_string(p.timestamp) == patternId) {
                p.cavitation_type = type;
                p.expert_note = expertNote;
                p.is_verified_by_expert = verified;
                p.last_updated = currentTimestampMs();
                return true;
            }
        }
        return false;
    }

    std::vector<TurbineConfig> getTurbineConfigs() const override {
        return turbineConfigs_;
    }

    std::vector<CavitationState> getCavitationState(uint8_t turbineId) const override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        std::vector<CavitationState> result;
        for (uint8_t bid = 1; bid <= BLADE_COUNT; ++bid) {
            size_t idx = (turbineId - 1) * BLADE_COUNT + (bid - 1);
            if (idx < cavitationCache_.size()) {
                result.push_back(cavitationCache_[idx]);
            }
        }
        return result;
    }

    std::vector<LifeAssessment> getLifeAssessment(uint8_t turbineId, uint8_t bladeId) const override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        std::vector<LifeAssessment> result;
        size_t idx = (turbineId - 1) * BLADE_COUNT + std::max(bladeId, uint8_t(1)) - 1;
        if (idx < lifeCache_.size()) {
            result.push_back(lifeCache_[idx]);
        }
        return result;
    }

    std::vector<AlarmLog> getActiveAlarms(uint8_t turbineId) const override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        std::vector<AlarmLog> result;
        for (const auto& a : alarmCache_) {
            if (turbineId == 0 || a.turbine_id == turbineId) {
                if (!a.acknowledged) result.push_back(a);
            }
        }
        return result;
    }

    bool acknowledgeAlarm(const std::string& alarmId, const std::string& user) override {
        std::lock_guard<std::mutex> lk(dataMutex_);
        for (auto& a : alarmCache_) {
            if (a.alarm_id == alarmId) {
                a.acknowledged = true;
                a.acknowledged_at = currentTimestampMs();
                a.acknowledged_by = user;
                return true;
            }
        }
        return false;
    }

    bool suppressAlarm(const std::string& alarmId, uint32_t seconds) override {
        return acknowledgeAlarm(alarmId, "system-supress");
    }

private:
    void pollIPC() {
        while (running_ && g_running) {
            IPCMessageCavitation cavMsg{};
            size_t cCount = 0;
            while (cavitationQueue_ && cavitationQueue_->pop(cavMsg) && cCount < 500) {
                if (cavMsg.turbine_id > 0 && cavMsg.turbine_id <= TURBINE_COUNT &&
                    cavMsg.blade_id > 0 && cavMsg.blade_id <= BLADE_COUNT) {
                    size_t idx = (cavMsg.turbine_id - 1) * BLADE_COUNT + (cavMsg.blade_id - 1);
                    std::lock_guard<std::mutex> lk(dataMutex_);
                    if (idx >= cavitationCache_.size()) cavitationCache_.resize(idx + 1);
                    CavitationState& s = cavitationCache_[idx];
                    s.timestamp = cavMsg.timestamp;
                    s.turbine_id = cavMsg.turbine_id;
                    s.blade_id = cavMsg.blade_id;
                    s.cavitation_stage = static_cast<CavitationStage>(cavMsg.cavitation_stage);
                    s.cavitation_intensity = cavMsg.cavitation_intensity;
                    s.confidence = cavMsg.confidence;
                    s.model_type = static_cast<ModelType>(cavMsg.model_type);
                    s.anomaly_score = cavMsg.anomaly_score;
                    s.reconstruction_error = cavMsg.reconstruction_error;
                }
                cCount++;
            }

            IPCMessageLife lifeMsg{};
            size_t lCount = 0;
            while (lifeQueue_ && lifeQueue_->pop(lifeMsg) && lCount < 500) {
                if (lifeMsg.turbine_id > 0 && lifeMsg.turbine_id <= TURBINE_COUNT &&
                    lifeMsg.blade_id > 0 && lifeMsg.blade_id <= BLADE_COUNT) {
                    size_t idx = (lifeMsg.turbine_id - 1) * BLADE_COUNT + (lifeMsg.blade_id - 1);
                    std::lock_guard<std::mutex> lk(dataMutex_);
                    if (idx >= lifeCache_.size()) lifeCache_.resize(idx + 1);
                    LifeAssessment& l = lifeCache_[idx];
                    l.timestamp = lifeMsg.timestamp;
                    l.turbine_id = lifeMsg.turbine_id;
                    l.blade_id = lifeMsg.blade_id;
                    l.cumulative_damage = lifeMsg.cumulative_damage;
                    l.remaining_life_hours = lifeMsg.remaining_life_hours;
                    l.remaining_life_days = lifeMsg.remaining_life_days;
                    l.miner_sum = lifeMsg.miner_sum;
                    l.fatigue_damage = lifeMsg.fatigue_damage;
                    l.cavitation_damage = lifeMsg.cavitation_damage;
                }
                lCount++;
            }

            IPCMessageAlarm alarmMsg{};
            size_t aCount = 0;
            while (alarmQueue_ && alarmQueue_->pop(alarmMsg) && aCount < 100) {
                std::lock_guard<std::mutex> lk(dataMutex_);
                AlarmLog a;
                a.timestamp = alarmMsg.timestamp;
                a.alarm_id = alarmMsg.alarm_id;
                a.turbine_id = alarmMsg.turbine_id;
                a.blade_id = alarmMsg.blade_id;
                a.alarm_type = static_cast<AlarmType>(alarmMsg.alarm_type);
                a.alarm_level = static_cast<AlarmLevel>(alarmMsg.alarm_level);
                a.alarm_message = alarmMsg.alarm_message;
                a.threshold_value = alarmMsg.threshold_value;
                a.actual_value = alarmMsg.actual_value;
                a.iec61850_pushed = alarmMsg.iec61850_pushed > 0;
                a.acknowledged = alarmMsg.acknowledged > 0;
                a.maintenance_suggestion = alarmMsg.maintenance_suggestion;
                a.acknowledged_at = alarmMsg.acknowledged_at;
                a.acknowledged_by = alarmMsg.acknowledged_by;
                auto it = std::find_if(alarmCache_.begin(), alarmCache_.end(),
                    [&a](const AlarmLog& x) { return x.alarm_id == a.alarm_id; });
                if (it == alarmCache_.end()) {
                    alarmCache_.push_back(a);
                    if (alarmCache_.size() > 1000) alarmCache_.erase(alarmCache_.begin());
                }
                aCount++;
            }

            // Feature 1: CONTROL channel
            IPCMessageControl ctrlMsg{};
            size_t ctrlCount = 0;
            while (controlQueue_ && controlQueue_->pop(ctrlMsg) && ctrlCount < 100) {
                if (ctrlMsg.turbine_id > 0 && ctrlMsg.turbine_id <= TURBINE_COUNT) {
                    std::lock_guard<std::mutex> lk(dataMutex_);
                    size_t idx = ctrlMsg.turbine_id - 1;
                    if (idx >= controlCache_.size()) controlCache_.resize(idx + 1);
                    TurbineControlStatus& c = controlCache_[idx];
                    c.turbine_id = ctrlMsg.turbine_id;
                    c.mode = static_cast<ControlMode>(ctrlMsg.control_mode);
                    c.cavitation_avoidance_enabled = ctrlMsg.cavitation_avoidance_enabled > 0;
                    c.guide_vane_opening_deg = ctrlMsg.guide_vane_opening_deg;
                    c.target_power_mw = ctrlMsg.target_power_mw;
                    c.current_head_m = ctrlMsg.current_head_m;
                    c.current_flow_m3s = ctrlMsg.current_flow_m3s;
                    c.predicted_efficiency = ctrlMsg.predicted_efficiency;
                    c.predicted_cavitation_risk = ctrlMsg.predicted_cavitation_risk;
                    c.mpc_cost_value = ctrlMsg.mpc_cost_value;
                    c.timestamp = ctrlMsg.timestamp;
                    c.action_desc = ctrlMsg.control_action_desc;
                    c.control_signals.assign(ctrlMsg.control_signal, ctrlMsg.control_signal + 8);
                    c.horizon_states.assign(ctrlMsg.horizon_states, ctrlMsg.horizon_states + 32);
                }
                ctrlCount++;
            }

            // Feature 2: ROBOT_PLAN channel
            IPCMessageRobotPlan robotMsg{};
            size_t robotCount = 0;
            while (robotQueue_ && robotQueue_->pop(robotMsg) && robotCount < 50) {
                std::lock_guard<std::mutex> lk(dataMutex_);
                RobotRepairTask rt{};
                rt.timestamp = robotMsg.timestamp;
                rt.turbine_id = robotMsg.turbine_id;
                rt.robot_status = static_cast<RobotStatus>(robotMsg.robot_status);
                rt.repair_mode = static_cast<RepairMode>(robotMsg.repair_mode);
                rt.estimated_duration_ms = robotMsg.estimated_duration_ms;
                rt.total_repair_area_cm2 = robotMsg.total_repair_area_cm2;
                rt.total_weld_volume_cm3 = robotMsg.total_weld_volume_cm3;
                rt.blade_damage_map.assign(robotMsg.blade_damage_map, robotMsg.blade_damage_map + 90);
                rt.current_waypoint_idx = 0;
                for (uint8_t i = 0; i < 15 && robotMsg.target_blade_ids[i] > 0; ++i) {
                    rt.target_blade_ids.push_back(robotMsg.target_blade_ids[i]);
                }
                rt.repair_sequence = robotMsg.repair_sequence;
                rt.robot_pos[0] = 0; rt.robot_pos[1] = 0; rt.robot_pos[2] = 0;
                auto it = std::find_if(robotCache_.begin(), robotCache_.end(),
                    [&rt](const RobotRepairTask& x) {
                        return x.turbine_id == rt.turbine_id &&
                               x.robot_status != RobotStatus::COMPLETED &&
                               x.robot_status != RobotStatus::IDLE;
                    });
                if (it == robotCache_.end()) {
                    robotCache_.push_back(rt);
                    if (robotCache_.size() > 100) robotCache_.erase(robotCache_.begin());
                } else {
                    *it = rt;
                }
                robotCount++;
            }

            // Feature 3: SCHEDULE channel
            IPCMessageSchedule schedMsg{};
            size_t schedCount = 0;
            while (scheduleQueue_ && scheduleQueue_->pop(schedMsg) && schedCount < 20) {
                std::lock_guard<std::mutex> lk(dataMutex_);
                scheduleCache_.timestamp = schedMsg.timestamp;
                scheduleCache_.schedule_id = schedMsg.schedule_id;
                scheduleCache_.status = static_cast<ScheduleStatus>(schedMsg.optimization_status);
                scheduleCache_.horizon_s = schedMsg.scheduling_horizon_s;
                scheduleCache_.target_total_power_mw = schedMsg.target_total_power_mw;
                scheduleCache_.current_total_power_mw = schedMsg.current_total_power_mw;
                scheduleCache_.optimized_efficiency_pct = schedMsg.optimized_efficiency_pct;
                scheduleCache_.cavitation_risk_reduction_pct = schedMsg.cavitation_risk_reduction_pct;
                scheduleCache_.mip_objective_value = schedMsg.mip_objective_value;
                scheduleCache_.note = schedMsg.schedule_note;
                scheduleCache_.units.clear();
                for (uint8_t i = 0; i < 6; ++i) {
                    UnitSchedule u{};
                    u.turbine_id = i + 1;
                    u.power_mw = schedMsg.unit_power_mw[i];
                    u.efficiency_pct = schedMsg.unit_efficiency[i];
                    u.cavitation_risk = schedMsg.unit_cavitation_risk[i];
                    u.operating_hours = schedMsg.unit_operating_hours[i];
                    u.is_active = (schedMsg.active_units_mask & (1u << i)) != 0;
                    scheduleCache_.units.push_back(u);
                }
                scheduleCache_.constraint_slack.assign(schedMsg.constraint_slack, schedMsg.constraint_slack + 8);
                schedCount++;
            }

            // Feature 4: DIAGNOSIS channel
            IPCMessageDiagnosis diagMsg{};
            size_t diagCount = 0;
            while (diagnosisQueue_ && diagnosisQueue_->pop(diagMsg) && diagCount < 200) {
                std::lock_guard<std::mutex> lk(dataMutex_);
                DiagnosisResult d{};
                d.timestamp = diagMsg.timestamp;
                d.turbine_id = diagMsg.turbine_id;
                d.sensor_id = diagMsg.sensor_id;
                d.cavitation_type = static_cast<CavitationType>(diagMsg.cavitation_type);
                d.status = static_cast<DiagnosisStatus>(diagMsg.diagnosis_status);
                d.cluster_label = diagMsg.cluster_label;
                d.is_known_pattern = diagMsg.is_known_pattern > 0;
                d.feature_embedding.assign(diagMsg.feature_embedding, diagMsg.feature_embedding + 32);
                d.pattern_similarity.assign(diagMsg.pattern_similarity, diagMsg.pattern_similarity + 4);
                d.confidence_scores.assign(diagMsg.confidence_scores, diagMsg.confidence_scores + 4);
                d.centroid_distance = diagMsg.centroid_distance;
                d.silhouette_score = diagMsg.silhouette_score;
                d.cluster_purity = diagMsg.cluster_purity;
                d.cavitation_type_name = diagMsg.cavitation_type_name;
                d.expert_note = diagMsg.expert_note;
                diagnosisCache_.push_back(d);
                if (diagnosisCache_.size() > 2000) diagnosisCache_.erase(diagnosisCache_.begin(), diagnosisCache_.end() - 2000);

                // auto register top patterns
                if (d.status == DiagnosisStatus::COMPLETED) {
                    auto pit = std::find_if(patternCache_.begin(), patternCache_.end(),
                        [&d](const AcousticPattern& p) {
                            return p.cavitation_type == d.cavitation_type && p.sample_count > 0;
                        });
                    if (pit == patternCache_.end()) {
                        AcousticPattern p{};
                        p.timestamp = d.timestamp;
                        p.cavitation_type = d.cavitation_type;
                        p.pattern_name = d.cavitation_type_name;
                        p.embedding = d.feature_embedding;
                        p.centroid = d.feature_embedding;
                        p.sample_count = 1;
                        p.silhouette_score = d.silhouette_score;
                        patternCache_.push_back(p);
                    } else {
                        pit->sample_count++;
                        pit->last_updated = d.timestamp;
                    }
                }
                diagCount++;
            }

            if (cCount == 0 && lCount == 0 && aCount == 0 &&
                ctrlCount == 0 && robotCount == 0 && schedCount == 0 && diagCount == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
    }

    void initDefaultCaches() {
        std::lock_guard<std::mutex> lk(dataMutex_);
        controlCache_.resize(TURBINE_COUNT);
        for (uint8_t i = 0; i < TURBINE_COUNT; ++i) {
            controlCache_[i].turbine_id = i + 1;
            controlCache_[i].mode = ControlMode::MPC_OPTIMAL;
            controlCache_[i].cavitation_avoidance_enabled = true;
            controlCache_[i].guide_vane_opening_deg = 50.0f + i * 2.0f;
            controlCache_[i].target_power_mw = 500.0f + i * 30.0f;
            controlCache_[i].current_head_m = 120.0f;
            controlCache_[i].current_flow_m3s = 620.0f;
            controlCache_[i].predicted_efficiency = 0.94f;
            controlCache_[i].predicted_cavitation_risk = 0.1f + i * 0.03f;
            controlCache_[i].timestamp = currentTimestampMs();
        }
        scheduleCache_.timestamp = currentTimestampMs();
        scheduleCache_.status = ScheduleStatus::CONVERGED;
        scheduleCache_.horizon_s = 24 * 3600;
        scheduleCache_.target_total_power_mw = 3000.0f;
        scheduleCache_.current_total_power_mw = 3000.0f;
        scheduleCache_.optimized_efficiency_pct = 94.0f;
        scheduleCache_.cavitation_risk_reduction_pct = 25.0f;
        scheduleCache_.schedule_id = 0;
        for (uint8_t i = 0; i < 6; ++i) {
            UnitSchedule u{};
            u.turbine_id = i + 1;
            u.is_active = true;
            u.power_mw = 500.0f + i * 30.0f;
            u.efficiency_pct = 92.0f + i * 0.5f;
            u.cavitation_risk = 0.1f + i * 0.03f;
            u.operating_hours = 10000.0f + i * 500.0f;
            scheduleCache_.units.push_back(u);
        }
    }

    std::vector<TurbineConfig> buildTurbineConfigs() {
        std::vector<TurbineConfig> configs;
        for (uint8_t i = 1; i <= TURBINE_COUNT; ++i) {
            TurbineConfig tc{};
            tc.turbine_id = i;
            tc.turbine_name = std::to_string(i) + "号水轮机";
            tc.turbine_type = "Francis";
            tc.rated_power = 700.0f;
            tc.rated_head = 120.0f;
            tc.rated_flow = 650.0f;
            tc.rated_speed = 107.1f;
            tc.blade_count = BLADE_COUNT;
            tc.material = "13Cr4Ni";
            tc.ultimate_tensile_strength = 750.0f;
            tc.fatigue_limit = 250.0f;
            tc.fracture_toughness = 60.0f;
            tc.cavitation_threshold = 0.3f;
            tc.vibration_threshold = 0.3f;
            tc.expected_life_hours = 200000.0f;
            configs.push_back(tc);
        }
        return configs;
    }

    std::vector<TurbineConfig> turbineConfigs_;
    mutable std::mutex dataMutex_;
    std::vector<CavitationState> cavitationCache_;
    std::vector<LifeAssessment> lifeCache_;
    std::vector<AlarmLog> alarmCache_;

    // Feature 1: MPC Control
    std::vector<TurbineControlStatus> controlCache_;
    std::unique_ptr<SharedMemorySPSC<IPCMessageControl>> controlQueue_;

    // Feature 2: Robot Repair
    std::vector<RobotRepairTask> robotCache_;
    std::unique_ptr<SharedMemorySPSC<IPCMessageRobotPlan>> robotQueue_;

    // Feature 3: Unit Scheduler
    PlantSchedule scheduleCache_;
    std::unique_ptr<SharedMemorySPSC<IPCMessageSchedule>> scheduleQueue_;

    // Feature 4: Acoustic Diagnosis
    std::vector<AcousticPattern> patternCache_;
    std::vector<DiagnosisResult> diagnosisCache_;
    std::unique_ptr<SharedMemorySPSC<IPCMessageDiagnosis>> diagnosisQueue_;

    std::atomic<bool> running_{false};
    std::thread pollThread_;
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
        std::cerr << "[api_gateway] Warning: Failed to load config" << std::endl;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const Config& cfg = getConfig();

    auto provider = std::make_shared<GatewayDataProvider>();
    if (!provider->init()) {
        std::cerr << "[api_gateway] Init data provider failed" << std::endl;
        return 1;
    }

    auto apiServer = std::make_unique<APIServer>(cfg.api_host, cfg.api_port, provider);
    if (!apiServer->start()) {
        std::cerr << "[api_gateway] API server start failed" << std::endl;
        return 1;
    }

    std::cout << "========================================" << std::endl;
    std::cout << "[api_gateway] Started successfully" << std::endl;
    std::cout << "  API: http://" << cfg.api_host << ":" << cfg.api_port << std::endl;
    std::cout << "  -- Legacy Channels --" << std::endl;
    std::cout << "  IPC CAVITATION -> channel 2 (consumer)" << std::endl;
    std::cout << "  IPC LIFE -> channel 4 (consumer)" << std::endl;
    std::cout << "  IPC ALARM -> channel 5 (consumer)" << std::endl;
    std::cout << "  -- New Feature Channels (v2.0) --" << std::endl;
    std::cout << "  IPC CONTROL -> channel 6 (consumer, F1:MPC)" << std::endl;
    std::cout << "  IPC ROBOT_PLAN -> channel 7 (consumer, F2:Robot)" << std::endl;
    std::cout << "  IPC SCHEDULE -> channel 8 (consumer, F3:Scheduler)" << std::endl;
    std::cout << "  IPC DIAGNOSIS -> channel 9 (consumer, F4:Diagnosis)" << std::endl;
    std::cout << "========================================" << std::endl;

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    provider->stop();
    apiServer->stop();
    std::cout << std::endl << "[api_gateway] Shutdown complete" << std::endl;
    return 0;
}
