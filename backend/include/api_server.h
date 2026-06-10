#pragma once

#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include <unordered_map>
#include "data_structures.h"

namespace turbine_monitor {

struct HttpRequest {
    std::string method;
    std::string path;
    std::unordered_map<std::string, std::string> params;
    std::unordered_map<std::string, std::string> headers;
    std::string body;
};

struct HttpResponse {
    int statusCode;
    std::string contentType;
    std::string body;
    std::unordered_map<std::string, std::string> headers;
};

using ApiHandler = std::function<HttpResponse(const HttpRequest&)>;

class DataProvider {
public:
    virtual ~DataProvider() = default;

    virtual std::vector<CavitationState> getCavitationState(
        uint8_t turbineId, uint64_t startTime = 0, uint64_t endTime = 0) = 0;

    virtual std::vector<LifeAssessment> getLifeAssessment(
        uint8_t turbineId, uint8_t bladeId, uint64_t startTime = 0, uint64_t endTime = 0) = 0;

    virtual std::vector<SpectrumFeatures> getSpectrumData(
        uint8_t turbineId, uint8_t sensorId, uint64_t startTime = 0, uint64_t endTime = 0) = 0;

    virtual std::vector<AlarmLog> getActiveAlarms(uint8_t turbineId = 0) = 0;

    virtual std::vector<TurbineConfig> getTurbineConfigs() = 0;

    virtual bool acknowledgeAlarm(const std::string& alarmId, const std::string& user) = 0;

    virtual bool suppressAlarm(uint8_t turbineId, uint8_t bladeId,
                                AlarmType type, uint32_t durationMs) = 0;

    // ==================== Feature 1: MPC Control ====================
    struct TurbineControlStatus {
        uint8_t turbine_id;
        ControlMode mode;
        bool cavitation_avoidance_enabled;
        float guide_vane_opening_deg;
        float target_power_mw;
        float current_head_m;
        float current_flow_m3s;
        float predicted_efficiency;
        float predicted_cavitation_risk;
        float mpc_cost_value;
        std::vector<float> control_signals;
        std::vector<float> horizon_states;
        std::string action_desc;
        uint64_t timestamp;
    };

    virtual std::vector<TurbineControlStatus> getControlStatus() = 0;
    virtual bool setControlCommand(uint8_t turbineId, ControlMode mode,
                                    float guideVaneDeg, float targetPowerMw,
                                    bool cavAvoidanceEnable,
                                    const std::vector<float>& weightsOverride) = 0;

    // ==================== Feature 2: Robot Repair ====================
    virtual std::vector<RobotRepairTask> getRobotTasks() = 0;
    virtual bool triggerRobotPlan(uint8_t turbineId, RepairMode mode,
                                   const std::vector<uint8_t>& targetBlades) = 0;
    virtual bool cancelRobotTask(const std::string& taskId) = 0;
    virtual RobotRepairTask getRobotTaskStatus(const std::string& taskId) = 0;

    // ==================== Feature 3: Unit Scheduler ====================
    virtual PlantSchedule getCurrentSchedule() = 0;
    virtual bool runOptimization(float targetTotalPowerMw, uint32_t horizonHours) = 0;
    virtual bool executeSchedule(uint8_t scheduleId) = 0;

    // ==================== Feature 4: Acoustic Diagnosis ====================
    virtual std::vector<AcousticPattern> getAcousticPatterns() = 0;
    virtual std::vector<DiagnosisResult> getDiagnosisResults(uint32_t limit = 50) = 0;
    virtual bool labelPattern(const std::string& patternId, CavitationType type,
                               const std::string& expertNote, bool verified) = 0;
};

class APIServer {
public:
    APIServer(const std::string& host, uint16_t port,
              std::shared_ptr<DataProvider> dataProvider);
    ~APIServer();

    bool start();
    void stop();
    bool isRunning() const;

    void registerHandler(const std::string& path, ApiHandler handler);

private:
    void serverLoop();
    HttpResponse handleRequest(const HttpRequest& request);

    HttpResponse getTurbineList(const HttpRequest& req);
    HttpResponse getTurbineDetail(const HttpRequest& req);
    HttpResponse getCavitationState(const HttpRequest& req);
    HttpResponse getLifeAssessment(const HttpRequest& req);
    HttpResponse getSpectrumData(const HttpRequest& req);
    HttpResponse getWaterfallData(const HttpRequest& req);
    HttpResponse getActiveAlarms(const HttpRequest& req);
    HttpResponse getAlarmHistory(const HttpRequest& req);
    HttpResponse acknowledgeAlarm(const HttpRequest& req);
    HttpResponse suppressAlarm(const HttpRequest& req);
    HttpResponse getSystemStatus(const HttpRequest& req);

    // Feature 1: MPC Control
    HttpResponse getControlStatus(const HttpRequest& req);
    HttpResponse postControlCommand(const HttpRequest& req);

    // Feature 2: Robot Repair
    HttpResponse getRobotTasks(const HttpRequest& req);
    HttpResponse getRobotTaskStatus(const HttpRequest& req);
    HttpResponse triggerRobotPlan(const HttpRequest& req);
    HttpResponse cancelRobotTask(const HttpRequest& req);

    // Feature 3: Unit Scheduler
    HttpResponse getCurrentSchedule(const HttpRequest& req);
    HttpResponse runScheduleOptimization(const HttpRequest& req);
    HttpResponse executeSchedule(const HttpRequest& req);

    // Feature 4: Acoustic Diagnosis
    HttpResponse getAcousticPatterns(const HttpRequest& req);
    HttpResponse getDiagnosisResults(const HttpRequest& req);
    HttpResponse labelPattern(const HttpRequest& req);

    std::string jsonError(const std::string& message, int code);
    std::string cavitationStateToJson(const CavitationState& state);
    std::string lifeAssessmentToJson(const LifeAssessment& assessment);
    std::string spectrumToJson(const SpectrumFeatures& features);
    std::string alarmToJson(const AlarmLog& alarm);
    std::string turbineConfigToJson(const TurbineConfig& config);
    std::string controlStatusToJson(const DataProvider::TurbineControlStatus& c);
    std::string robotTaskToJson(const RobotRepairTask& t);
    std::string scheduleToJson(const PlantSchedule& s);
    std::string patternToJson(const AcousticPattern& p);
    std::string diagnosisToJson(const DiagnosisResult& d);

    std::string host_;
    uint16_t port_;
    std::atomic<bool> running_;
    std::thread serverThread_;

    std::unordered_map<std::string, ApiHandler> handlers_;
    std::shared_ptr<DataProvider> dataProvider_;

#ifdef _WIN32
    SOCKET serverSocket_;
#else
    int serverSocket_;
#endif

    mutable std::mutex mutex_;
};

}
