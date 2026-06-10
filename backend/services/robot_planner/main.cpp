#include <iostream>
#include <memory>
#include <csignal>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <mutex>
#include <deque>
#include <sstream>
#include <iomanip>

#include "../../include/config.h"
#include "../../include/robot_repair_planner.h"
#include "../../include/clickhouse_client.h"
#include "../../../common/include/ipc_queue.h"
#include "../../../common/include/service_base.h"
#include "../../../common/include/metrics.h"

using namespace turbine_monitor;

std::atomic<bool> g_running(true);

void signalHandler(int) {
    std::cout << "\n[robot_planner] Received shutdown signal" << std::endl;
    g_running = false;
}

static const char* robotStatusStr(RobotStatus s) {
    switch (s) {
        case RobotStatus::IDLE:        return "IDLE";
        case RobotStatus::PLANNING:    return "PLANNING";
        case RobotStatus::DEPLOYING:   return "DEPLOYING";
        case RobotStatus::INSPECTING:  return "INSPECTING";
        case RobotStatus::POLISHING:   return "POLISHING";
        case RobotStatus::WELDING:     return "WELDING";
        case RobotStatus::RETURNING:   return "RETURNING";
        case RobotStatus::COMPLETED:   return "COMPLETED";
        case RobotStatus::FAULT:       return "FAULT";
        default: return "UNKNOWN";
    }
}

class RobotPlannerService : public ServiceBase {
public:
    RobotPlannerService() : ServiceBase("robot_planner", 7) {}

    bool init(const Config& config) override {
        config_ = config;

        planner_ = std::make_unique<RobotPlannerFacade>();
        planner_->setConfig(config_.robot);

        clickhouseClient_ = std::make_unique<ClickHouseClient>(
            config_.clickhouse_host, config_.clickhouse_port,
            config_.clickhouse_user, config_.clickhouse_pass,
            config_.clickhouse_db);

        lifeQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageLife>>(
            IPCChannel::LIFE, IPC_DEFAULT_CAPACITY, false);
        if (!lifeQueue_->open()) {
            std::cerr << "[" << name_ << "] Failed to open LIFE IPC (consumer)" << std::endl;
            return false;
        }

        alarmQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageAlarm>>(
            IPCChannel::ALARM, IPC_DEFAULT_CAPACITY, false);
        if (!alarmQueue_->open()) {
            std::cerr << "[" << name_ << "] Failed to open ALARM IPC (consumer)" << std::endl;
            return false;
        }

        planQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageRobotPlan>>(
            IPCChannel::ROBOT_PLAN, IPC_DEFAULT_CAPACITY, true);
        if (!planQueue_->open()) {
            std::cerr << "[" << name_ << "] Failed to open ROBOT_PLAN IPC (producer)" << std::endl;
            return false;
        }

        try {
            auto& reg = MetricsRegistry::instance();
            auto& hist_fam = prometheus::BuildHistogram()
                .Name("robot_task_duration_histogram")
                .Help("Robot repair task duration distribution")
                .Register(*reg.registry_);
            taskDurationHist_ = &hist_fam.Add({}, prometheus::Histogram::BucketBoundaries{
                10.0, 30.0, 60.0, 120.0, 300.0, 600.0, 1800.0, 3600.0, 7200.0
            });

            auto& cnt_fam = prometheus::BuildCounter()
                .Name("repair_area_total_counter")
                .Help("Total repair area in cm2")
                .Register(*reg.registry_);
            repairAreaCounter_ = &cnt_fam.Add({});

            auto& gauge_fam = prometheus::BuildGauge()
                .Name("waypoint_execution_gauge")
                .Help("Current waypoint execution index")
                .Register(*reg.registry_);
            waypointGauge_ = &gauge_fam.Add({});
        } catch (const std::exception& e) {
            std::cerr << "[" << name_ << "] Metrics init failed: " << e.what() << std::endl;
        }

        lastLifeCache_.resize((TURBINE_COUNT + 1) * (BLADE_COUNT + 1));

        return true;
    }

    bool start() override {
        if (!clickhouseClient_->connect()) {
            std::cerr << "[" << name_ << "] ClickHouse connect failed, running without DB" << std::endl;
        }
        clickhouseClient_->startBatchWriter(BATCH_WRITE_SIZE, 1000);

        lifeThread_ = std::thread([this]() { this->lifeConsumeLoop(); });
        alarmThread_ = std::thread([this]() { this->alarmConsumeLoop(); });
        plannerThread_ = std::thread([this]() { this->plannerLoop(); });
        simThread_ = std::thread([this]() { this->simulationLoop(); });
        statsThread_ = std::thread([this]() { this->statsLoop(); });

        std::cout << "========================================" << std::endl;
        std::cout << "[robot_planner] Started successfully" << std::endl;
        std::cout << "  IPC LIFE -> channel 4 (consumer)" << std::endl;
        std::cout << "  IPC ALARM -> channel 5 (consumer)" << std::endl;
        std::cout << "  IPC ROBOT_PLAN -> channel 7 (producer)" << std::endl;
        std::cout << "  Auto trigger life < " << config_.robot.auto_trigger_life_pct << "%" << std::endl;
        std::cout << "  Default mode: " << static_cast<int>(config_.robot.default_repair_mode) << std::endl;
        std::cout << "========================================" << std::endl;

        return ServiceBase::start();
    }

    void stop() override {
        ServiceBase::stop();
        if (lifeThread_.joinable()) lifeThread_.join();
        if (alarmThread_.joinable()) alarmThread_.join();
        if (plannerThread_.joinable()) plannerThread_.join();
        if (simThread_.joinable()) simThread_.join();
        if (statsThread_.joinable()) statsThread_.join();
        clickhouseClient_->stopBatchWriter();
        clickhouseClient_->disconnect();
        lifeQueue_->close();
        alarmQueue_->close();
        planQueue_->close();
    }

    void join() override {
        while (running_ && g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        stop();
    }

private:
    void lifeConsumeLoop() {
        IPCMessageLife lifeMsg{};
        while (g_running && running_) {
            size_t processed = 0;
            const size_t maxBatch = 200;
            while (processed < maxBatch && lifeQueue_->pop(lifeMsg) && g_running) {
                if (lifeMsg.turbine_id > 0 && lifeMsg.blade_id > 0 &&
                    lifeMsg.turbine_id <= TURBINE_COUNT && lifeMsg.blade_id <= BLADE_COUNT) {
                    size_t idx = (lifeMsg.turbine_id - 1) * BLADE_COUNT + (lifeMsg.blade_id - 1);
                    LifeAssessment la{};
                    la.timestamp = lifeMsg.timestamp;
                    la.turbine_id = lifeMsg.turbine_id;
                    la.blade_id = lifeMsg.blade_id;
                    la.cumulative_damage = lifeMsg.cumulative_damage;
                    la.remaining_life_hours = lifeMsg.remaining_life_hours;
                    la.remaining_life_days = lifeMsg.remaining_life_days;
                    la.miner_sum = lifeMsg.miner_sum;
                    la.fatigue_damage = lifeMsg.fatigue_damage;
                    la.cavitation_damage = lifeMsg.cavitation_damage;
                    la.stress_range = lifeMsg.stress_range;
                    la.cycle_count = lifeMsg.cycle_count;

                    lastLifeCache_[idx] = la;
                    planner_->heatMap().update(lifeMsg.turbine_id, lifeMsg.blade_id,
                        lifeMsg.cumulative_damage,
                        std::clamp(lifeMsg.cavitation_damage, 0.0f, 1.0f));

                    {
                        std::lock_guard<std::mutex> lk(taskQueueMutex_);
                        pendingLife_.push_back(la);
                    }
                }
                {
                    std::lock_guard<std::mutex> lk(statsMutex_);
                    stats_.lifeConsumed++;
                }
                processed++;
            }
            if (processed == 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
        }
    }

    void alarmConsumeLoop() {
        IPCMessageAlarm alarmMsg{};
        while (g_running && running_) {
            size_t processed = 0;
            const size_t maxBatch = 50;
            while (processed < maxBatch && alarmQueue_->pop(alarmMsg) && g_running) {
                if (alarmMsg.alarm_type == static_cast<uint8_t>(AlarmType::LIFE_CRITICAL)
                    || alarmMsg.alarm_level == static_cast<uint8_t>(AlarmLevel::EMERGENCY)) {
                    if (alarmMsg.turbine_id > 0 && alarmMsg.turbine_id <= TURBINE_COUNT) {
                        std::vector<uint8_t> blades;
                        if (alarmMsg.blade_id > 0 && alarmMsg.blade_id <= BLADE_COUNT) {
                            blades.push_back(alarmMsg.blade_id);
                        } else {
                            for (uint8_t b = 1; b <= BLADE_COUNT; ++b) blades.push_back(b);
                        }
                        {
                            std::lock_guard<std::mutex> lk(taskQueueMutex_);
                            urgentTasks_.push_back({alarmMsg.turbine_id, blades,
                                config_.robot.default_repair_mode, currentTimestampMs()});
                        }
                        std::cout << "\n[robot_planner] EMERGENCY repair triggered: turbine="
                                  << (int)alarmMsg.turbine_id
                                  << " blade=" << (int)alarmMsg.blade_id << std::endl;
                    }
                }
                {
                    std::lock_guard<std::mutex> lk(statsMutex_);
                    stats_.alarmConsumed++;
                }
                processed++;
            }
            if (processed == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

    struct PendingTask {
        uint8_t turbine_id;
        std::vector<uint8_t> blade_ids;
        RepairMode mode;
        uint64_t requested_at;
    };

    void plannerLoop() {
        uint64_t lastAutoCheck = 0;
        while (g_running && running_) {
            uint64_t now = currentTimestampMs();
            PendingTask task_to_plan{};
            bool has_task = false;

            {
                std::lock_guard<std::mutex> lk(taskQueueMutex_);
                if (!urgentTasks_.empty()) {
                    task_to_plan = urgentTasks_.front();
                    urgentTasks_.pop_front();
                    has_task = true;
                } else if (now - lastAutoCheck >= 5000) {
                    lastAutoCheck = now;
                    std::vector<LifeAssessment> life_batch;
                    life_batch.swap(pendingLife_);
                    if (!life_batch.empty() && planner_->autoTriggerCheck(life_batch)) {
                        uint8_t worst_turbine = 1;
                        float worst_dmg = 0;
                        for (auto& la : life_batch) {
                            if (la.cumulative_damage > worst_dmg) {
                                worst_dmg = la.cumulative_damage;
                                worst_turbine = la.turbine_id;
                            }
                        }
                        task_to_plan.turbine_id = worst_turbine;
                        task_to_plan.mode = config_.robot.default_repair_mode;
                        task_to_plan.requested_at = now;
                        has_task = true;
                        std::cout << "\n[robot_planner] AUTO-trigger: turbine=" << (int)worst_turbine
                                  << " damage=" << worst_dmg << std::endl;
                    }
                }
            }

            if (has_task) {
                RobotRepairTask task;
                {
                    std::lock_guard<std::mutex> lk(activeTaskMutex_);
                    if (activeTask_ && activeTask_->robot_status != RobotStatus::COMPLETED
                        && activeTask_->robot_status != RobotStatus::FAULT
                        && activeTask_->robot_status != RobotStatus::IDLE) {
                        taskQueueMutex_.lock();
                        urgentTasks_.push_front(task_to_plan);
                        taskQueueMutex_.unlock();
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                        continue;
                    }

                    if (task_to_plan.blade_ids.empty()) {
                        task = planner_->planFullInspection(task_to_plan.turbine_id);
                    } else {
                        task = planner_->planRepair(task_to_plan.turbine_id,
                            task_to_plan.blade_ids, task_to_plan.mode);
                    }
                    task.robot_status = RobotStatus::DEPLOYING;
                    task_started_at_ = now;
                    activeTask_ = std::make_unique<RobotRepairTask>(task);
                }
                publishPlan(task);
                insertTaskToDB(task);
                {
                    std::lock_guard<std::mutex> lk(statsMutex_);
                    stats_.tasksPlanned++;
                }
                std::cout << "\n[robot_planner] Planned task: T" << (int)task.turbine_id
                          << " blades=" << task.target_blade_ids.size()
                          << " mode=" << static_cast<int>(task.repair_mode)
                          << " dur=" << task.estimated_duration_ms << "ms" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void simulationLoop() {
        uint64_t lastTick = currentTimestampMs();
        while (g_running && running_) {
            uint64_t now = currentTimestampMs();
            uint32_t dt = static_cast<uint32_t>(std::min<uint64_t>(now - lastTick, 100));
            lastTick = now;

            bool has_active = false;
            {
                std::lock_guard<std::mutex> lk(activeTaskMutex_);
                if (activeTask_ && activeTask_->robot_status != RobotStatus::COMPLETED
                    && activeTask_->robot_status != RobotStatus::FAULT) {
                    has_active = true;
                    RobotWaypoint cur = planner_->simulateExecution(*activeTask_, dt);
                    if (waypointGauge_) {
                        waypointGauge_->Set(static_cast<double>(activeTask_->current_waypoint_idx));
                    }
                    IPCMessageRobotPlan msg{};
                    fillPlanMsg(msg, *activeTask_, cur);
                    if (!planQueue_->push(msg)) {
                        std::lock_guard<std::mutex> slk(statsMutex_);
                        stats_.planDropped++;
                    } else {
                        std::lock_guard<std::mutex> slk(statsMutex_);
                        stats_.planProduced++;
                    }

                    if (activeTask_->robot_status == RobotStatus::COMPLETED) {
                        float dur_s = (now - task_started_at_) / 1000.0f;
                        if (taskDurationHist_) taskDurationHist_->Observe(dur_s);
                        if (repairAreaCounter_) {
                            repairAreaCounter_->Increment(static_cast<double>(activeTask_->total_repair_area_cm2));
                        }
                        updateTaskDB(*activeTask_);
                        std::cout << "\n[robot_planner] Task COMPLETED in " << dur_s << "s, area="
                                  << activeTask_->total_repair_area_cm2 << "cm2" << std::endl;
                    }
                }
            }

            if (!has_active) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
    }

    void statsLoop() {
        uint64_t lastPrint = 0;
        while (g_running && running_) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            uint64_t now = currentTimestampMs();
            if (now - lastPrint < 5000) continue;
            lastPrint = now;

            auto planS = planQueue_->getStats();
            std::string status_str = "IDLE";
            {
                std::lock_guard<std::mutex> lk(activeTaskMutex_);
                if (activeTask_) {
                    status_str = robotStatusStr(activeTask_->robot_status);
                }
            }

            std::lock_guard<std::mutex> lk(statsMutex_);
            std::cout << "\r[robot_planner] "
                      << "life_in=" << stats_.lifeConsumed
                      << " | alarm_in=" << stats_.alarmConsumed
                      << " | tasks=" << stats_.tasksPlanned
                      << " | plan_out=" << stats_.planProduced
                      << " | q=" << planS.currentSize
                      << " | drop=" << stats_.planDropped
                      << " | status=" << status_str
                      << std::flush;
        }
    }

    void publishPlan(const RobotRepairTask& task) {
        IPCMessageRobotPlan msg{};
        RobotWaypoint wp{};
        wp.x = task.robot_pos[0]; wp.y = task.robot_pos[1]; wp.z = task.robot_pos[2];
        wp.orientation_w = 1;
        fillPlanMsg(msg, task, wp);
        planQueue_->push(msg);
    }

    void fillPlanMsg(IPCMessageRobotPlan& msg, const RobotRepairTask& task,
                     const RobotWaypoint& cur_wp) {
        msg.timestamp = currentTimestampMs();
        msg.turbine_id = task.turbine_id;
        msg.blade_count = static_cast<uint8_t>(task.target_blade_ids.size());
        msg.robot_status = static_cast<uint8_t>(task.robot_status);
        msg.repair_mode = static_cast<uint8_t>(task.repair_mode);
        msg.estimated_duration_ms = task.estimated_duration_ms;
        msg.total_repair_area_cm2 = task.total_repair_area_cm2;
        msg.total_weld_volume_cm3 = task.total_weld_volume_cm3;

        std::memset(msg.target_blade_ids, 0, sizeof(msg.target_blade_ids));
        for (size_t i = 0; i < task.target_blade_ids.size() && i < 15; ++i) {
            msg.target_blade_ids[i] = task.target_blade_ids[i];
        }

        std::memset(msg.waypoint_path, 0, sizeof(msg.waypoint_path));
        msg.waypoint_path[0] = cur_wp.x;
        msg.waypoint_path[1] = cur_wp.y;
        msg.waypoint_path[2] = cur_wp.z;
        msg.waypoint_path[3] = cur_wp.orientation_w;
        msg.waypoint_path[4] = cur_wp.orientation_x;
        msg.waypoint_path[5] = cur_wp.orientation_y;
        msg.waypoint_path[6] = cur_wp.orientation_z;
        msg.waypoint_path[7] = static_cast<float>(task.current_waypoint_idx);

        std::memset(msg.blade_damage_map, 0, sizeof(msg.blade_damage_map));
        for (size_t i = 0; i < task.blade_damage_map.size() && i < 90; ++i) {
            msg.blade_damage_map[i] = task.blade_damage_map[i];
        }

        std::memset(msg.weld_trajectory, 0, sizeof(msg.weld_trajectory));
        for (size_t i = 0; i < task.weld_trajectory.size() && i < 16; ++i) {
            msg.weld_trajectory[i * 8 + 0] = task.weld_trajectory[i].x;
            msg.weld_trajectory[i * 8 + 1] = task.weld_trajectory[i].y;
            msg.weld_trajectory[i * 8 + 2] = task.weld_trajectory[i].z;
            msg.weld_trajectory[i * 8 + 3] = task.weld_trajectory[i].orientation_w;
            msg.weld_trajectory[i * 8 + 4] = task.weld_trajectory[i].orientation_x;
            msg.weld_trajectory[i * 8 + 5] = task.weld_trajectory[i].orientation_y;
            msg.weld_trajectory[i * 8 + 6] = task.weld_trajectory[i].orientation_z;
            msg.weld_trajectory[i * 8 + 7] = task.weld_trajectory[i].speed_mm_s;
        }

        std::memset(msg.polish_trajectory, 0, sizeof(msg.polish_trajectory));
        for (size_t i = 0; i < task.polish_trajectory.size() && i < 16; ++i) {
            msg.polish_trajectory[i * 8 + 0] = task.polish_trajectory[i].x;
            msg.polish_trajectory[i * 8 + 1] = task.polish_trajectory[i].y;
            msg.polish_trajectory[i * 8 + 2] = task.polish_trajectory[i].z;
            msg.polish_trajectory[i * 8 + 3] = task.polish_trajectory[i].orientation_w;
            msg.polish_trajectory[i * 8 + 4] = task.polish_trajectory[i].orientation_x;
            msg.polish_trajectory[i * 8 + 5] = task.polish_trajectory[i].orientation_y;
            msg.polish_trajectory[i * 8 + 6] = task.polish_trajectory[i].orientation_z;
            msg.polish_trajectory[i * 8 + 7] = task.polish_trajectory[i].speed_mm_s;
        }

        std::memset(msg.repair_sequence, 0, sizeof(msg.repair_sequence));
        size_t seq_len = task.repair_sequence.size();
        if (seq_len >= sizeof(msg.repair_sequence)) seq_len = sizeof(msg.repair_sequence) - 1;
        std::memcpy(msg.repair_sequence, task.repair_sequence.c_str(), seq_len);
    }

    void insertTaskToDB(const RobotRepairTask& task) {
        try {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(4);
            oss << "INSERT INTO robot_repair_tasks ("
                << "timestamp, turbine_id, robot_status, repair_mode, "
                << "target_blade_ids, estimated_duration_ms, total_repair_area_cm2, "
                << "total_weld_volume_cm3, repair_sequence, "
                << "robot_pos_x, robot_pos_y, robot_pos_z, waypoint_idx) VALUES ("
                << task.timestamp << ", "
                << static_cast<int>(task.turbine_id) << ", "
                << static_cast<int>(task.robot_status) << ", "
                << static_cast<int>(task.repair_mode) << ", "
                << "'[";
            for (size_t i = 0; i < task.target_blade_ids.size(); ++i) {
                if (i) oss << ",";
                oss << static_cast<int>(task.target_blade_ids[i]);
            }
            oss << "]', "
                << task.estimated_duration_ms << ", "
                << task.total_repair_area_cm2 << ", "
                << task.total_weld_volume_cm3 << ", '"
                << task.repair_sequence << "', "
                << task.robot_pos[0] << ", "
                << task.robot_pos[1] << ", "
                << task.robot_pos[2] << ", "
                << task.current_waypoint_idx << ")";
            clickhouseClient_->executeQuery(oss.str());
        } catch (...) {}
    }

    void updateTaskDB(const RobotRepairTask& task) {
        try {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(4);
            oss << "INSERT INTO robot_repair_tasks ("
                << "timestamp, turbine_id, robot_status, repair_mode, "
                << "target_blade_ids, estimated_duration_ms, total_repair_area_cm2, "
                << "total_weld_volume_cm3, repair_sequence, "
                << "robot_pos_x, robot_pos_y, robot_pos_z, waypoint_idx) VALUES ("
                << currentTimestampMs() << ", "
                << static_cast<int>(task.turbine_id) << ", "
                << static_cast<int>(task.robot_status) << ", "
                << static_cast<int>(task.repair_mode) << ", "
                << "'[";
            for (size_t i = 0; i < task.target_blade_ids.size(); ++i) {
                if (i) oss << ",";
                oss << static_cast<int>(task.target_blade_ids[i]);
            }
            oss << "]', "
                << task.estimated_duration_ms << ", "
                << task.total_repair_area_cm2 << ", "
                << task.total_weld_volume_cm3 << ", '"
                << task.repair_sequence << "', "
                << task.robot_pos[0] << ", "
                << task.robot_pos[1] << ", "
                << task.robot_pos[2] << ", "
                << task.current_waypoint_idx << ")";
            clickhouseClient_->executeQuery(oss.str());
        } catch (...) {}
    }

    Config config_;
    std::unique_ptr<RobotPlannerFacade> planner_;
    std::unique_ptr<ClickHouseClient> clickhouseClient_;

    std::unique_ptr<SharedMemorySPSC<IPCMessageLife>> lifeQueue_;
    std::unique_ptr<SharedMemorySPSC<IPCMessageAlarm>> alarmQueue_;
    std::unique_ptr<SharedMemorySPSC<IPCMessageRobotPlan>> planQueue_;

    std::vector<LifeAssessment> lastLifeCache_;

    std::thread lifeThread_;
    std::thread alarmThread_;
    std::thread plannerThread_;
    std::thread simThread_;
    std::thread statsThread_;

    std::mutex taskQueueMutex_;
    std::deque<PendingTask> urgentTasks_;
    std::vector<LifeAssessment> pendingLife_;

    std::mutex activeTaskMutex_;
    std::unique_ptr<RobotRepairTask> activeTask_;
    uint64_t task_started_at_ = 0;

    prometheus::Histogram* taskDurationHist_ = nullptr;
    prometheus::Counter* repairAreaCounter_ = nullptr;
    prometheus::Gauge* waypointGauge_ = nullptr;

    struct ServiceStats {
        uint64_t lifeConsumed = 0;
        uint64_t alarmConsumed = 0;
        uint64_t tasksPlanned = 0;
        uint64_t planProduced = 0;
        uint64_t planDropped = 0;
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
        std::cerr << "[robot_planner] Warning: Failed to load config, using defaults" << std::endl;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const Config& cfg = getConfig();

    try {
        init_logging("robot_planner", "info", "./logs");
        MetricsRegistry::instance().init("0.0.0.0:9107");
    } catch (const std::exception& e) {
        std::cerr << "[robot_planner] Init logging/metrics failed: " << e.what() << std::endl;
    }

    auto service = std::make_unique<RobotPlannerService>();
    if (!service->init(cfg)) {
        std::cerr << "[robot_planner] Init failed" << std::endl;
        return 1;
    }

    if (!service->start()) {
        std::cerr << "[robot_planner] Start failed" << std::endl;
        return 1;
    }

    service->join();
    std::cout << std::endl << "[robot_planner] Shutdown complete" << std::endl;
    return 0;
}
