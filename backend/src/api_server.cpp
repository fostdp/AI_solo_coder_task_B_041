#include "api_server.h"
#include <iostream>
#include <sstream>
#include <cstring>
#include <json/json.h>
#include <algorithm>

#ifdef _WIN32
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

namespace turbine_monitor {

APIServer::APIServer(const std::string& host, uint16_t port,
                     std::shared_ptr<DataProvider> dataProvider)
    : host_(host), port_(port), running_(false),
      serverSocket_(INVALID_SOCKET),
      dataProvider_(std::move(dataProvider)) {
}

APIServer::~APIServer() {
    stop();
}

bool APIServer::isRunning() const {
    return running_;
}

void APIServer::registerHandler(const std::string& path, ApiHandler handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_[path] = std::move(handler);
}

std::string APIServer::jsonError(const std::string& message, int code) {
    Json::Value json;
    json["error"]["code"] = code;
    json["error"]["message"] = message;
    Json::StreamWriterBuilder writer;
    return Json::writeString(writer, json);
}

std::string APIServer::cavitationStateToJson(const CavitationState& state) {
    Json::Value json;
    json["timestamp"] = Json::Value::UInt64(state.timestamp);
    json["turbine_id"] = state.turbine_id;
    json["blade_id"] = state.blade_id;
    json["cavitation_stage"] = static_cast<int>(state.cavitation_stage);
    json["cavitation_intensity"] = state.cavitation_intensity;
    json["confidence"] = state.confidence;
    json["model_type"] = static_cast<int>(state.model_type);
    json["anomaly_score"] = state.anomaly_score;
    json["reconstruction_error"] = state.reconstruction_error;

    Json::Value features(Json::arrayValue);
    for (float f : state.feature_vector) {
        features.append(f);
    }
    json["feature_vector"] = features;

    Json::StreamWriterBuilder writer;
    return Json::writeString(writer, json);
}

std::string APIServer::lifeAssessmentToJson(const LifeAssessment& a) {
    Json::Value json;
    json["timestamp"] = Json::Value::UInt64(a.timestamp);
    json["turbine_id"] = a.turbine_id;
    json["blade_id"] = a.blade_id;
    json["cumulative_damage"] = a.cumulative_damage;
    json["remaining_life_hours"] = a.remaining_life_hours;
    json["remaining_life_days"] = a.remaining_life_days;
    json["miner_sum"] = a.miner_sum;
    json["fatigue_damage"] = a.fatigue_damage;
    json["cavitation_damage"] = a.cavitation_damage;
    json["material_constant_k"] = a.material_constant_k;
    json["material_constant_m"] = a.material_constant_m;
    json["stress_range"] = a.stress_range;
    json["cycle_count"] = a.cycle_count;
    json["assessment_method"] = a.assessment_method;

    Json::StreamWriterBuilder writer;
    return Json::writeString(writer, json);
}

std::string APIServer::spectrumToJson(const SpectrumFeatures& f) {
    Json::Value json;
    json["timestamp"] = Json::Value::UInt64(f.timestamp);
    json["turbine_id"] = f.turbine_id;
    json["sensor_type"] = static_cast<int>(f.sensor_type);
    json["sensor_id"] = f.sensor_id;
    json["blade_id"] = f.blade_id;
    json["peak_frequency"] = f.peak_frequency;
    json["rms_value"] = f.rms_value;
    json["crest_factor"] = f.crest_factor;
    json["kurtosis"] = f.kurtosis;
    json["skewness"] = f.skewness;
    json["band_energy_low"] = f.band_energy_low;
    json["band_energy_mid"] = f.band_energy_mid;
    json["band_energy_high"] = f.band_energy_high;
    json["harmonic_ratio"] = f.harmonic_ratio;
    json["spectral_centroid"] = f.spectral_centroid;
    json["spectral_bandwidth"] = f.spectral_bandwidth;

    Json::StreamWriterBuilder writer;
    return Json::writeString(writer, json);
}

std::string APIServer::alarmToJson(const AlarmLog& a) {
    Json::Value json;
    json["timestamp"] = Json::Value::UInt64(a.timestamp);
    json["alarm_id"] = a.alarm_id;
    json["turbine_id"] = a.turbine_id;
    json["blade_id"] = a.blade_id;
    json["alarm_type"] = static_cast<int>(a.alarm_type);
    json["alarm_level"] = static_cast<int>(a.alarm_level);
    json["alarm_message"] = a.alarm_message;
    json["threshold_value"] = a.threshold_value;
    json["actual_value"] = a.actual_value;
    json["iec61850_pushed"] = a.iec61850_pushed;
    json["acknowledged"] = a.acknowledged;
    json["maintenance_suggestion"] = a.maintenance_suggestion;
    json["acknowledged_at"] = Json::Value::UInt64(a.acknowledged_at);
    json["acknowledged_by"] = a.acknowledged_by;

    Json::StreamWriterBuilder writer;
    return Json::writeString(writer, json);
}

std::string APIServer::turbineConfigToJson(const TurbineConfig& c) {
    Json::Value json;
    json["turbine_id"] = c.turbine_id;
    json["turbine_name"] = c.turbine_name;
    json["turbine_type"] = c.turbine_type;
    json["rated_power"] = c.rated_power;
    json["rated_head"] = c.rated_head;
    json["rated_flow"] = c.rated_flow;
    json["rated_speed"] = c.rated_speed;
    json["blade_count"] = c.blade_count;
    json["material"] = c.material;
    json["ultimate_tensile_strength"] = c.ultimate_tensile_strength;
    json["fatigue_limit"] = c.fatigue_limit;
    json["fracture_toughness"] = c.fracture_toughness;
    json["cavitation_threshold"] = c.cavitation_threshold;
    json["vibration_threshold"] = c.vibration_threshold;
    json["expected_life_hours"] = c.expected_life_hours;

    Json::StreamWriterBuilder writer;
    return Json::writeString(writer, json);
}

HttpResponse APIServer::getTurbineList(const HttpRequest& req) {
    auto configs = dataProvider_->getTurbineConfigs();
    Json::Value json(Json::arrayValue);
    for (const auto& c : configs) {
        Json::Value item;
        item["turbine_id"] = c.turbine_id;
        item["turbine_name"] = c.turbine_name;
        item["turbine_type"] = c.turbine_type;
        item["rated_power"] = c.rated_power;
        json.append(item);
    }
    Json::StreamWriterBuilder writer;
    return {200, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::getTurbineDetail(const HttpRequest& req) {
    auto it = req.params.find("id");
    if (it == req.params.end()) {
        return {400, "application/json", jsonError("Missing turbine id", 400), {}};
    }
    uint8_t turbineId = static_cast<uint8_t>(std::stoi(it->second));

    auto configs = dataProvider_->getTurbineConfigs();
    auto configIt = std::find_if(configs.begin(), configs.end(),
        [turbineId](const TurbineConfig& c) { return c.turbine_id == turbineId; });

    if (configIt == configs.end()) {
        return {404, "application/json", jsonError("Turbine not found", 404), {}};
    }

    Json::Value json;
    json["config"] = Json::Value(Json::objectValue);
    json["config"] = Json::parse(turbineConfigToJson(*configIt));

    auto cavitation = dataProvider_->getCavitationState(turbineId);
    Json::Value cavArray(Json::arrayValue);
    for (const auto& c : cavitation) {
        cavArray.append(Json::parse(cavitationStateToJson(c)));
    }
    json["cavitation"] = cavArray;

    auto life = dataProvider_->getLifeAssessment(turbineId, 0);
    Json::Value lifeArray(Json::arrayValue);
    for (const auto& l : life) {
        lifeArray.append(Json::parse(lifeAssessmentToJson(l)));
    }
    json["life_assessment"] = lifeArray;

    auto alarms = dataProvider_->getActiveAlarms(turbineId);
    Json::Value alarmArray(Json::arrayValue);
    for (const auto& a : alarms) {
        alarmArray.append(Json::parse(alarmToJson(a)));
    }
    json["active_alarms"] = alarmArray;

    Json::StreamWriterBuilder writer;
    return {200, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::getCavitationState(const HttpRequest& req) {
    auto it = req.params.find("turbine_id");
    if (it == req.params.end()) {
        return {400, "application/json", jsonError("Missing turbine_id", 400), {}};
    }
    uint8_t turbineId = static_cast<uint8_t>(std::stoi(it->second));

    auto states = dataProvider_->getCavitationState(turbineId);

    Json::Value json(Json::arrayValue);
    for (const auto& s : states) {
        json.append(Json::parse(cavitationStateToJson(s)));
    }

    Json::StreamWriterBuilder writer;
    return {200, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::getLifeAssessment(const HttpRequest& req) {
    auto turbineIt = req.params.find("turbine_id");
    auto bladeIt = req.params.find("blade_id");
    if (turbineIt == req.params.end()) {
        return {400, "application/json", jsonError("Missing turbine_id", 400), {}};
    }
    uint8_t turbineId = static_cast<uint8_t>(std::stoi(turbineIt->second));
    uint8_t bladeId = bladeIt != req.params.end() ? static_cast<uint8_t>(std::stoi(bladeIt->second)) : 0;

    auto assessments = dataProvider_->getLifeAssessment(turbineId, bladeId);

    Json::Value json(Json::arrayValue);
    for (const auto& a : assessments) {
        json.append(Json::parse(lifeAssessmentToJson(a)));
    }

    Json::StreamWriterBuilder writer;
    return {200, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::getSpectrumData(const HttpRequest& req) {
    auto turbineIt = req.params.find("turbine_id");
    auto sensorIt = req.params.find("sensor_id");
    if (turbineIt == req.params.end() || sensorIt == req.params.end()) {
        return {400, "application/json", jsonError("Missing parameters", 400), {}};
    }
    uint8_t turbineId = static_cast<uint8_t>(std::stoi(turbineIt->second));
    uint8_t sensorId = static_cast<uint8_t>(std::stoi(sensorIt->second));

    auto spectra = dataProvider_->getSpectrumData(turbineId, sensorId);

    Json::Value json(Json::arrayValue);
    for (const auto& s : spectra) {
        json.append(Json::parse(spectrumToJson(s)));
    }

    Json::StreamWriterBuilder writer;
    return {200, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::getWaterfallData(const HttpRequest& req) {
    auto turbineIt = req.params.find("turbine_id");
    auto sensorIt = req.params.find("sensor_id");
    auto limitIt = req.params.find("limit");

    if (turbineIt == req.params.end() || sensorIt == req.params.end()) {
        return {400, "application/json", jsonError("Missing parameters", 400), {}};
    }

    uint8_t turbineId = static_cast<uint8_t>(std::stoi(turbineIt->second));
    uint8_t sensorId = static_cast<uint8_t>(std::stoi(sensorIt->second));
    int limit = limitIt != req.params.end() ? std::stoi(limitIt->second) : 100;

    auto spectra = dataProvider_->getSpectrumData(turbineId, sensorId);
    if (spectra.size() > static_cast<size_t>(limit)) {
        spectra.resize(limit);
    }

    Json::Value json(Json::arrayValue);
    for (const auto& s : spectra) {
        Json::Value row;
        row["timestamp"] = Json::Value::UInt64(s.timestamp);
        row["rms"] = s.rms_value;
        row["peak_freq"] = s.peak_frequency;
        row["kurtosis"] = s.kurtosis;
        row["crest"] = s.crest_factor;
        row["band_low"] = s.band_energy_low;
        row["band_mid"] = s.band_energy_mid;
        row["band_high"] = s.band_energy_high;
        json.append(row);
    }

    Json::StreamWriterBuilder writer;
    return {200, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::getActiveAlarms(const HttpRequest& req) {
    auto it = req.params.find("turbine_id");
    uint8_t turbineId = it != req.params.end() ? static_cast<uint8_t>(std::stoi(it->second)) : 0;

    auto alarms = dataProvider_->getActiveAlarms(turbineId);

    Json::Value json(Json::arrayValue);
    for (const auto& a : alarms) {
        json.append(Json::parse(alarmToJson(a)));
    }

    Json::StreamWriterBuilder writer;
    return {200, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::getAlarmHistory(const HttpRequest& req) {
    return {200, "application/json", "[]", {}};
}

HttpResponse APIServer::acknowledgeAlarm(const HttpRequest& req) {
    Json::CharReaderBuilder builder;
    Json::Value body;
    std::string errs;
    std::istringstream ss(req.body);
    if (!Json::parseFromStream(builder, ss, &body, &errs)) {
        return {400, "application/json", jsonError("Invalid JSON body", 400), {}};
    }

    if (!body.isMember("alarm_id") || !body.isMember("user")) {
        return {400, "application/json", jsonError("Missing alarm_id or user", 400), {}};
    }

    std::string alarmId = body["alarm_id"].asString();
    std::string user = body["user"].asString();

    bool success = dataProvider_->acknowledgeAlarm(alarmId, user);

    Json::Value json;
    json["success"] = success;
    json["message"] = success ? "Alarm acknowledged" : "Failed to acknowledge alarm";

    Json::StreamWriterBuilder writer;
    return {success ? 200 : 500, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::suppressAlarm(const HttpRequest& req) {
    Json::CharReaderBuilder builder;
    Json::Value body;
    std::string errs;
    std::istringstream ss(req.body);
    if (!Json::parseFromStream(builder, ss, &body, &errs)) {
        return {400, "application/json", jsonError("Invalid JSON body", 400), {}};
    }

    if (!body.isMember("turbine_id") || !body.isMember("blade_id") ||
        !body.isMember("alarm_type") || !body.isMember("duration_ms")) {
        return {400, "application/json", jsonError("Missing parameters", 400), {}};
    }

    uint8_t turbineId = static_cast<uint8_t>(body["turbine_id"].asUInt());
    uint8_t bladeId = static_cast<uint8_t>(body["blade_id"].asUInt());
    AlarmType type = static_cast<AlarmType>(body["alarm_type"].asUInt());
    uint32_t durationMs = body["duration_ms"].asUInt();

    bool success = dataProvider_->suppressAlarm(turbineId, bladeId, type, durationMs);

    Json::Value json;
    json["success"] = success;

    Json::StreamWriterBuilder writer;
    return {success ? 200 : 500, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::getSystemStatus(const HttpRequest& req) {
    Json::Value json;
    json["status"] = "running";
    json["timestamp"] = Json::Value::UInt64(currentTimestampMs());

    Json::StreamWriterBuilder writer;
    return {200, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::handleRequest(const HttpRequest& request) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = handlers_.find(request.path);
    if (it != handlers_.end()) {
        return it->second(request);
    }

    if (request.path == "/api/turbines") {
        return getTurbineList(request);
    } else if (request.path == "/api/turbine/detail") {
        return getTurbineDetail(request);
    } else if (request.path == "/api/cavitation") {
        return getCavitationState(request);
    } else if (request.path == "/api/life") {
        return getLifeAssessment(request);
    } else if (request.path == "/api/spectrum") {
        return getSpectrumData(request);
    } else if (request.path == "/api/waterfall") {
        return getWaterfallData(request);
    } else if (request.path == "/api/alarms/active") {
        return getActiveAlarms(request);
    } else if (request.path == "/api/alarms/history") {
        return getAlarmHistory(request);
    } else if (request.path == "/api/alarms/acknowledge") {
        return acknowledgeAlarm(request);
    } else if (request.path == "/api/alarms/suppress") {
        return suppressAlarm(request);
    } else if (request.path == "/api/status") {
        return getSystemStatus(request);
    }

    if (request.method == "GET" && request.path.rfind("/api/robot/task/", 0) == 0) {
        std::string taskId = request.path.substr(std::string("/api/robot/task/").size());
        HttpRequest modifiedReq = request;
        modifiedReq.params["taskId"] = taskId;
        return getRobotTaskStatus(modifiedReq);
    }
    if (request.method == "POST" && request.path.rfind("/api/robot/plan/", 0) == 0) {
        std::string turbineIdStr = request.path.substr(std::string("/api/robot/plan/").size());
        HttpRequest modifiedReq = request;
        modifiedReq.params["turbineId"] = turbineIdStr;
        return triggerRobotPlan(modifiedReq);
    }
    if (request.method == "POST" && request.path.rfind("/api/robot/cancel/", 0) == 0) {
        std::string taskId = request.path.substr(std::string("/api/robot/cancel/").size());
        HttpRequest modifiedReq = request;
        modifiedReq.params["taskId"] = taskId;
        return cancelRobotTask(modifiedReq);
    }
    if (request.method == "POST" && request.path.rfind("/api/schedule/execute/", 0) == 0) {
        std::string idStr = request.path.substr(std::string("/api/schedule/execute/").size());
        HttpRequest modifiedReq = request;
        modifiedReq.params["id"] = idStr;
        return executeSchedule(modifiedReq);
    }

    return {404, "application/json", jsonError("Not found", 404), {}};
}

void APIServer::serverLoop() {
    while (running_) {
        sockaddr_in clientAddr{};
#ifdef _WIN32
        int addrLen = sizeof(clientAddr);
#else
        socklen_t addrLen = sizeof(clientAddr);
#endif

        SOCKET clientSocket = accept(serverSocket_, (sockaddr*)&clientAddr, &addrLen);
        if (clientSocket == INVALID_SOCKET) {
            if (running_) {
                std::cerr << "Accept failed" << std::endl;
            }
            continue;
        }

        char buffer[4096];
        int bytesReceived = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);
        if (bytesReceived <= 0) {
#ifdef _WIN32
            closesocket(clientSocket);
#else
            close(clientSocket);
#endif
            continue;
        }

        buffer[bytesReceived] = '\0';
        std::string requestStr(buffer);

        HttpRequest request;
        size_t methodEnd = requestStr.find(' ');
        if (methodEnd != std::string::npos) {
            request.method = requestStr.substr(0, methodEnd);
            size_t pathEnd = requestStr.find(' ', methodEnd + 1);
            if (pathEnd != std::string::npos) {
                request.path = requestStr.substr(methodEnd + 1, pathEnd - methodEnd - 1);

                size_t queryStart = request.path.find('?');
                if (queryStart != std::string::npos) {
                    std::string query = request.path.substr(queryStart + 1);
                    request.path = request.path.substr(0, queryStart);

                    size_t pos = 0;
                    while (pos < query.size()) {
                        size_t eq = query.find('=', pos);
                        if (eq == std::string::npos) break;
                        size_t amp = query.find('&', eq);
                        std::string key = query.substr(pos, eq - pos);
                        std::string val = (amp != std::string::npos) ?
                            query.substr(eq + 1, amp - eq - 1) : query.substr(eq + 1);
                        request.params[key] = val;
                        pos = (amp != std::string::npos) ? amp + 1 : query.size();
                    }
                }
            }
        }

        size_t bodyStart = requestStr.find("\r\n\r\n");
        if (bodyStart != std::string::npos) {
            request.body = requestStr.substr(bodyStart + 4);
        }

        HttpResponse response = handleRequest(request);

        std::stringstream responseStream;
        responseStream << "HTTP/1.1 " << response.statusCode << " OK\r\n";
        responseStream << "Content-Type: " << response.contentType << "\r\n";
        responseStream << "Access-Control-Allow-Origin: *\r\n";
        responseStream << "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n";
        responseStream << "Access-Control-Allow-Headers: Content-Type\r\n";
        responseStream << "Content-Length: " << response.body.size() << "\r\n";
        responseStream << "Connection: close\r\n\r\n";
        responseStream << response.body;

        std::string responseStr = responseStream.str();
        send(clientSocket, responseStr.c_str(), responseStr.size(), 0);

#ifdef _WIN32
        closesocket(clientSocket);
#else
        close(clientSocket);
#endif
    }
}

bool APIServer::start() {
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        return false;
    }
#endif

    serverSocket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (serverSocket_ == INVALID_SOCKET) {
        return false;
    }

    int opt = 1;
#ifdef _WIN32
    setsockopt(serverSocket_, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));
#else
    setsockopt(serverSocket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#endif

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port_);
    if (host_ == "0.0.0.0" || host_.empty()) {
        serverAddr.sin_addr.s_addr = INADDR_ANY;
    } else {
        inet_pton(AF_INET, host_.c_str(), &serverAddr.sin_addr);
    }

    if (bind(serverSocket_, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "Failed to bind API server to " << host_ << ":" << port_ << std::endl;
#ifdef _WIN32
        closesocket(serverSocket_);
        WSACleanup();
#else
        close(serverSocket_);
#endif
        return false;
    }

    if (listen(serverSocket_, 10) == SOCKET_ERROR) {
#ifdef _WIN32
        closesocket(serverSocket_);
        WSACleanup();
#else
        close(serverSocket_);
#endif
        return false;
    }

    running_ = true;
    serverThread_ = std::thread(&APIServer::serverLoop, this);

    registerHandler("/api/control/status", [this](const HttpRequest& req) { return getControlStatus(req); });
    registerHandler("/api/control/command", [this](const HttpRequest& req) { return postControlCommand(req); });
    registerHandler("/api/robot/tasks", [this](const HttpRequest& req) { return getRobotTasks(req); });
    registerHandler("/api/schedule/current", [this](const HttpRequest& req) { return getCurrentSchedule(req); });
    registerHandler("/api/schedule/run", [this](const HttpRequest& req) { return runScheduleOptimization(req); });
    registerHandler("/api/diagnosis/patterns", [this](const HttpRequest& req) { return getAcousticPatterns(req); });
    registerHandler("/api/diagnosis/latest", [this](const HttpRequest& req) { return getDiagnosisResults(req); });
    registerHandler("/api/diagnosis/label", [this](const HttpRequest& req) { return labelPattern(req); });

    std::cout << "API Server started on " << host_ << ":" << port_ << std::endl;
    return true;
}

void APIServer::stop() {
    running_ = false;

    if (serverSocket_ != INVALID_SOCKET) {
#ifdef _WIN32
        closesocket(serverSocket_);
        WSACleanup();
#else
        close(serverSocket_);
#endif
        serverSocket_ = INVALID_SOCKET;
    }

    if (serverThread_.joinable()) {
        serverThread_.join();
    }
}

std::string APIServer::controlStatusToJson(const DataProvider::TurbineControlStatus& c) {
    Json::Value json;
    json["turbine_id"] = c.turbine_id;
    json["mode"] = static_cast<int>(c.mode);
    json["cavitation_avoidance_enabled"] = c.cavitation_avoidance_enabled;
    json["guide_vane"] = c.guide_vane_opening_deg;
    json["target_power"] = c.target_power_mw;
    json["current_head_m"] = c.current_head_m;
    json["current_flow_m3s"] = c.current_flow_m3s;
    json["efficiency_pred"] = c.predicted_efficiency;
    json["cav_risk_pred"] = c.predicted_cavitation_risk;
    json["mpc_cost_value"] = c.mpc_cost_value;
    json["action_desc"] = c.action_desc;
    json["timestamp"] = Json::Value::UInt64(c.timestamp);

    Json::Value ctrlSignals(Json::arrayValue);
    for (float s : c.control_signals) {
        ctrlSignals.append(s);
    }
    json["control_signals"] = ctrlSignals;

    Json::Value horizonStates(Json::arrayValue);
    for (float h : c.horizon_states) {
        horizonStates.append(h);
    }
    json["horizon_states"] = horizonStates;

    Json::StreamWriterBuilder writer;
    return Json::writeString(writer, json);
}

std::string APIServer::robotTaskToJson(const RobotRepairTask& t) {
    Json::Value json;
    json["timestamp"] = Json::Value::UInt64(t.timestamp);
    json["turbine_id"] = t.turbine_id;
    json["task_status"] = static_cast<int>(t.robot_status);
    json["repair_mode"] = static_cast<int>(t.repair_mode);
    json["estimated_duration_ms"] = Json::Value::UInt64(t.estimated_duration_ms);
    json["total_repair_area_cm2"] = t.total_repair_area_cm2;
    json["total_weld_volume_cm3"] = t.total_weld_volume_cm3;
    json["repair_sequence"] = t.repair_sequence;
    json["current_waypoint_idx"] = t.current_waypoint_idx;

    Json::Value targetBlades(Json::arrayValue);
    for (uint8_t b : t.target_blade_ids) {
        targetBlades.append(b);
    }
    json["target_blades"] = targetBlades;

    Json::Value robotPos(Json::arrayValue);
    for (int i = 0; i < 3; i++) {
        robotPos.append(t.robot_pos[i]);
    }
    json["robot_position"] = robotPos;

    Json::Value waypoints(Json::arrayValue);
    for (const auto& wp : t.inspection_path) {
        Json::Value w;
        w["x"] = wp.x; w["y"] = wp.y; w["z"] = wp.z;
        w["orientation_w"] = wp.orientation_w;
        w["orientation_x"] = wp.orientation_x;
        w["orientation_y"] = wp.orientation_y;
        w["orientation_z"] = wp.orientation_z;
        w["speed_mm_s"] = wp.speed_mm_s;
        w["dwell_time_s"] = wp.dwell_time_s;
        w["action_type"] = wp.action_type;
        waypoints.append(w);
    }
    json["trajectory_waypoints"] = waypoints;

    Json::Value damageMap(Json::arrayValue);
    for (float d : t.blade_damage_map) {
        damageMap.append(d);
    }
    json["damage_heatmap"] = damageMap;

    Json::StreamWriterBuilder writer;
    return Json::writeString(writer, json);
}

std::string APIServer::scheduleToJson(const PlantSchedule& s) {
    Json::Value json;
    json["timestamp"] = Json::Value::UInt64(s.timestamp);
    json["schedule_id"] = s.schedule_id;
    json["status"] = static_cast<int>(s.status);
    json["horizon_hours"] = static_cast<float>(s.horizon_s) / 3600.0f;
    json["target_total_power_mw"] = s.target_total_power_mw;
    json["current_total_power_mw"] = s.current_total_power_mw;
    json["optimized_efficiency_pct"] = s.optimized_efficiency_pct;
    json["cavitation_risk_reduction_pct"] = s.cavitation_risk_reduction_pct;
    json["mip_objective_value"] = s.mip_objective_value;
    json["gap"] = s.mip_objective_value > 0 ? (s.constraint_slack.size() > 0 ? s.constraint_slack[0] : 0.0f) : 0.0f;
    json["converged"] = (s.status == ScheduleStatus::CONVERGED);
    json["note"] = s.note;

    Json::Value units(Json::arrayValue);
    for (const auto& u : s.units) {
        Json::Value unit;
        unit["turbine_id"] = u.turbine_id;
        unit["is_active"] = u.is_active;
        unit["power"] = u.power_mw;
        unit["efficiency"] = u.efficiency_pct;
        unit["cav_risk"] = u.cavitation_risk;
        unit["operating_hours"] = u.operating_hours;
        unit["startup_cost"] = u.startup_cost;
        unit["shutdown_cost"] = u.shutdown_cost;
        units.append(unit);
    }
    json["units"] = units;

    Json::Value slack(Json::arrayValue);
    for (float v : s.constraint_slack) {
        slack.append(v);
    }
    json["constraint_slack"] = slack;

    Json::StreamWriterBuilder writer;
    return Json::writeString(writer, json);
}

std::string APIServer::patternToJson(const AcousticPattern& p) {
    Json::Value json;
    json["timestamp"] = Json::Value::UInt64(p.timestamp);
    json["pattern_id"] = p.pattern_name;
    json["cavitation_type"] = static_cast<int>(p.cavitation_type);
    json["pattern_name"] = p.pattern_name;
    json["description"] = p.description;
    json["sample_count"] = p.sample_count;
    json["intra_cluster_variance"] = p.intra_cluster_variance;
    json["silhouette_score"] = p.silhouette_score;
    json["is_verified"] = p.is_verified_by_expert;
    json["expert_note"] = p.expert_note;
    json["last_updated"] = Json::Value::UInt64(p.last_updated);

    Json::Value embedding(Json::arrayValue);
    for (float e : p.embedding) {
        embedding.append(e);
    }
    json["embedding"] = embedding;

    Json::Value centroid(Json::arrayValue);
    for (float c : p.centroid) {
        centroid.append(c);
    }
    json["centroid"] = centroid;

    Json::Value samplesArr(Json::arrayValue);
    for (const auto& s : p.samples) {
        Json::Value sample(Json::arrayValue);
        for (float v : s) {
            sample.append(v);
        }
        samplesArr.append(sample);
    }
    json["samples"] = samplesArr;

    Json::StreamWriterBuilder writer;
    return Json::writeString(writer, json);
}

std::string APIServer::diagnosisToJson(const DiagnosisResult& d) {
    Json::Value json;
    json["timestamp"] = Json::Value::UInt64(d.timestamp);
    json["turbine_id"] = d.turbine_id;
    json["sensor_id"] = d.sensor_id;
    json["cavitation_type"] = static_cast<int>(d.cavitation_type);
    json["status"] = static_cast<int>(d.status);
    json["cluster_label"] = d.cluster_label;
    json["is_known_pattern"] = d.is_known_pattern;
    json["is_unknown"] = !d.is_known_pattern;
    json["centroid_distance"] = d.centroid_distance;
    json["silhouette_score"] = d.silhouette_score;
    json["cluster_purity"] = d.cluster_purity;
    json["cavitation_type_name"] = d.cavitation_type_name;
    json["expert_note"] = d.expert_note;
    json["analysis_latency_us"] = Json::Value::UInt64(d.analysis_latency_us);

    Json::Value embedding(Json::arrayValue);
    for (float e : d.embedding) {
        embedding.append(e);
    }
    json["embedding"] = embedding;

    Json::Value similarity(Json::arrayValue);
    for (float s : d.pattern_similarity) {
        similarity.append(s);
    }
    json["similarity"] = similarity;

    Json::Value confidence(Json::arrayValue);
    for (float c : d.confidence_scores) {
        confidence.append(c);
    }
    json["confidence"] = confidence;

    Json::StreamWriterBuilder writer;
    return Json::writeString(writer, json);
}

HttpResponse APIServer::getControlStatus(const HttpRequest& req) {
    auto statuses = dataProvider_->getControlStatus();
    Json::Value json(Json::arrayValue);
    for (const auto& s : statuses) {
        json.append(Json::parse(controlStatusToJson(s)));
    }
    Json::StreamWriterBuilder writer;
    return {200, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::postControlCommand(const HttpRequest& req) {
    Json::CharReaderBuilder builder;
    Json::Value body;
    std::string errs;
    std::istringstream ss(req.body);
    if (!Json::parseFromStream(builder, ss, &body, &errs)) {
        return {400, "application/json", jsonError("Invalid JSON body", 400), {}};
    }

    if (!body.isMember("turbine_id") || !body.isMember("mode") ||
        !body.isMember("guide_vane") || !body.isMember("target_power") ||
        !body.isMember("cav_avoidance_enable") || !body.isMember("weights")) {
        return {400, "application/json", jsonError("Missing required parameters", 400), {}};
    }

    uint8_t turbineId = static_cast<uint8_t>(body["turbine_id"].asUInt());
    ControlMode mode = static_cast<ControlMode>(body["mode"].asUInt());
    float guideVane = body["guide_vane"].asFloat();
    float targetPower = body["target_power"].asFloat();
    bool cavAvoidance = body["cav_avoidance_enable"].asBool();

    std::vector<float> weights;
    const Json::Value& wArr = body["weights"];
    if (!wArr.isArray() || wArr.size() != 4) {
        return {400, "application/json", jsonError("weights must be array of 4 elements", 400), {}};
    }
    for (const auto& w : wArr) {
        weights.push_back(w.asFloat());
    }

    bool success = dataProvider_->setControlCommand(turbineId, mode, guideVane, targetPower, cavAvoidance, weights);

    Json::Value json;
    json["success"] = success;
    json["message"] = success ? "Control command accepted" : "Failed to apply control command";

    Json::StreamWriterBuilder writer;
    return {success ? 200 : 500, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::getRobotTasks(const HttpRequest& req) {
    auto tasks = dataProvider_->getRobotTasks();
    Json::Value json(Json::arrayValue);
    for (const auto& t : tasks) {
        json.append(Json::parse(robotTaskToJson(t)));
    }
    Json::StreamWriterBuilder writer;
    return {200, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::getRobotTaskStatus(const HttpRequest& req) {
    auto it = req.params.find("taskId");
    if (it == req.params.end()) {
        return {400, "application/json", jsonError("Missing taskId", 400), {}};
    }
    std::string taskId = it->second;

    try {
        auto task = dataProvider_->getRobotTaskStatus(taskId);
        Json::Value json = Json::parse(robotTaskToJson(task));
        Json::StreamWriterBuilder writer;
        return {200, "application/json", Json::writeString(writer, json), {}};
    } catch (...) {
        return {404, "application/json", jsonError("Task not found", 404), {}};
    }
}

HttpResponse APIServer::triggerRobotPlan(const HttpRequest& req) {
    auto it = req.params.find("turbineId");
    if (it == req.params.end()) {
        return {400, "application/json", jsonError("Missing turbineId", 400), {}};
    }
    uint8_t turbineId = static_cast<uint8_t>(std::stoi(it->second));

    Json::CharReaderBuilder builder;
    Json::Value body;
    std::string errs;
    std::istringstream ss(req.body);
    if (!Json::parseFromStream(builder, ss, &body, &errs)) {
        return {400, "application/json", jsonError("Invalid JSON body", 400), {}};
    }

    if (!body.isMember("repair_mode") || !body.isMember("target_blades")) {
        return {400, "application/json", jsonError("Missing repair_mode or target_blades", 400), {}};
    }

    RepairMode mode = static_cast<RepairMode>(body["repair_mode"].asUInt());
    std::vector<uint8_t> targetBlades;
    const Json::Value& bladesArr = body["target_blades"];
    if (!bladesArr.isArray()) {
        return {400, "application/json", jsonError("target_blades must be array", 400), {}};
    }
    for (const auto& b : bladesArr) {
        targetBlades.push_back(static_cast<uint8_t>(b.asUInt()));
    }

    bool success = dataProvider_->triggerRobotPlan(turbineId, mode, targetBlades);

    Json::Value json;
    json["success"] = success;
    json["message"] = success ? "Robot planning triggered" : "Failed to trigger robot planning";

    Json::StreamWriterBuilder writer;
    return {success ? 200 : 500, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::cancelRobotTask(const HttpRequest& req) {
    auto it = req.params.find("taskId");
    if (it == req.params.end()) {
        return {400, "application/json", jsonError("Missing taskId", 400), {}};
    }
    std::string taskId = it->second;

    bool success = dataProvider_->cancelRobotTask(taskId);

    Json::Value json;
    json["success"] = success;
    json["message"] = success ? "Task cancelled" : "Failed to cancel task";

    Json::StreamWriterBuilder writer;
    return {success ? 200 : 500, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::getCurrentSchedule(const HttpRequest& req) {
    auto schedule = dataProvider_->getCurrentSchedule();
    Json::Value json = Json::parse(scheduleToJson(schedule));
    Json::StreamWriterBuilder writer;
    return {200, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::runScheduleOptimization(const HttpRequest& req) {
    Json::CharReaderBuilder builder;
    Json::Value body;
    std::string errs;
    std::istringstream ss(req.body);
    if (!Json::parseFromStream(builder, ss, &body, &errs)) {
        return {400, "application/json", jsonError("Invalid JSON body", 400), {}};
    }

    if (!body.isMember("target_total_power_mw") || !body.isMember("horizon_hours")) {
        return {400, "application/json", jsonError("Missing target_total_power_mw or horizon_hours", 400), {}};
    }

    float targetPower = body["target_total_power_mw"].asFloat();
    uint32_t horizonHours = body["horizon_hours"].asUInt();

    bool success = dataProvider_->runOptimization(targetPower, horizonHours);

    Json::Value json;
    json["success"] = success;
    json["message"] = success ? "Schedule optimization started" : "Failed to start optimization";

    Json::StreamWriterBuilder writer;
    return {success ? 200 : 500, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::executeSchedule(const HttpRequest& req) {
    auto it = req.params.find("id");
    if (it == req.params.end()) {
        return {400, "application/json", jsonError("Missing schedule id", 400), {}};
    }
    uint8_t scheduleId = static_cast<uint8_t>(std::stoi(it->second));

    bool success = dataProvider_->executeSchedule(scheduleId);

    Json::Value json;
    json["success"] = success;
    json["message"] = success ? "Schedule execution started" : "Failed to execute schedule";

    Json::StreamWriterBuilder writer;
    return {success ? 200 : 500, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::getAcousticPatterns(const HttpRequest& req) {
    auto patterns = dataProvider_->getAcousticPatterns();
    Json::Value json(Json::arrayValue);
    for (const auto& p : patterns) {
        json.append(Json::parse(patternToJson(p)));
    }
    Json::StreamWriterBuilder writer;
    return {200, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::getDiagnosisResults(const HttpRequest& req) {
    uint32_t limit = 50;
    auto it = req.params.find("limit");
    if (it != req.params.end()) {
        limit = static_cast<uint32_t>(std::stoi(it->second));
    }

    auto results = dataProvider_->getDiagnosisResults(limit);
    Json::Value json(Json::arrayValue);
    for (const auto& d : results) {
        json.append(Json::parse(diagnosisToJson(d)));
    }
    Json::StreamWriterBuilder writer;
    return {200, "application/json", Json::writeString(writer, json), {}};
}

HttpResponse APIServer::labelPattern(const HttpRequest& req) {
    Json::CharReaderBuilder builder;
    Json::Value body;
    std::string errs;
    std::istringstream ss(req.body);
    if (!Json::parseFromStream(builder, ss, &body, &errs)) {
        return {400, "application/json", jsonError("Invalid JSON body", 400), {}};
    }

    if (!body.isMember("pattern_id") || !body.isMember("cavitation_type") ||
        !body.isMember("expert_note") || !body.isMember("is_verified")) {
        return {400, "application/json", jsonError("Missing required parameters", 400), {}};
    }

    std::string patternId = body["pattern_id"].asString();
    CavitationType type = static_cast<CavitationType>(body["cavitation_type"].asUInt());
    std::string expertNote = body["expert_note"].asString();
    bool verified = body["is_verified"].asBool();

    bool success = dataProvider_->labelPattern(patternId, type, expertNote, verified);

    Json::Value json;
    json["success"] = success;
    json["message"] = success ? "Pattern labeled successfully" : "Failed to label pattern";

    Json::StreamWriterBuilder writer;
    return {success ? 200 : 500, "application/json", Json::writeString(writer, json), {}};
}

}
