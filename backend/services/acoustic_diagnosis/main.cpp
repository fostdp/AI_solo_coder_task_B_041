#include <iostream>
#include <memory>
#include <csignal>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <mutex>
#include <sstream>
#include <cstring>

#include "../../include/config.h"
#include "../../include/acoustic_diagnosis.h"
#include "../../include/clickhouse_client.h"
#include "../../../common/include/ipc_queue.h"
#include "../../../common/include/service_base.h"
#include "../../../common/include/metrics.h"

using namespace turbine_monitor;

std::atomic<bool> g_running(true);

void signalHandler(int) {
    std::cout << "\n[acoustic_diagnosis] Received shutdown signal" << std::endl;
    g_running = false;
}

static void strncpy_safe(char* dest, const std::string& src, size_t max_len) {
    if (!dest || max_len == 0) return;
    size_t copy_len = std::min(src.size(), max_len - 1);
    std::memcpy(dest, src.c_str(), copy_len);
    dest[copy_len] = '\0';
}

class AcousticDiagnosisService : public ServiceBase {
public:
    AcousticDiagnosisService() : ServiceBase("acoustic_diagnosis", 6) {}

    bool init(const Config& config) override {
        config_ = config;

        diagnosis_ = std::make_unique<AcousticDiagnosisFacade>();
        if (!diagnosis_->init(config_.diagnosis)) {
            std::cerr << "[" << name_ << "] Warning: Diagnosis init with defaults" << std::endl;
        }

        clickhouseClient_ = std::make_unique<ClickHouseClient>(
            config_.clickhouse_host, config_.clickhouse_port,
            config_.clickhouse_user, config_.clickhouse_pass,
            config_.clickhouse_db);

        featuresQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageFeatures>>(
            IPCChannel::FEATURES, IPC_DEFAULT_CAPACITY, false);
        if (!featuresQueue_->open()) {
            std::cerr << "[" << name_ << "] Failed to open FEATURES IPC (consumer)" << std::endl;
            return false;
        }

        diagnosisQueue_ = std::make_unique<SharedMemorySPSC<IPCMessageDiagnosis>>(
            IPCChannel::DIAGNOSIS, IPC_DEFAULT_CAPACITY, true);
        if (!diagnosisQueue_->open()) {
            std::cerr << "[" << name_ << "] Failed to open DIAGNOSIS IPC (producer)" << std::endl;
            return false;
        }

        initPrometheusMetrics();

        return true;
    }

    bool start() override {
        if (!clickhouseClient_->connect()) {
            std::cerr << "[" << name_ << "] ClickHouse connect failed, running without DB" << std::endl;
        }
        clickhouseClient_->startBatchWriter(BATCH_WRITE_SIZE, 1000);

        diagnosisThread_ = std::thread([this]() { this->diagnosisLoop(); });
        updateThread_ = std::thread([this]() { this->periodicUpdateLoop(); });
        statsThread_ = std::thread([this]() { this->statsLoop(); });

        std::cout << "========================================" << std::endl;
        std::cout << "[acoustic_diagnosis] Started successfully" << std::endl;
        std::cout << "  IPC FEATURES -> channel 1 (consumer)" << std::endl;
        std::cout << "  IPC DIAGNOSIS -> channel 9 (producer)" << std::endl;
        std::cout << "  Pattern count: 4 default (Cloud/Sheet/Super/Vortex)" << std::endl;
        std::cout << "  Update interval: " << config_.diagnosis.update_interval_s << "s" << std::endl;
        std::cout << "  Similarity threshold: " << config_.diagnosis.similarity_threshold << std::endl;
        std::cout << "========================================" << std::endl;

        return ServiceBase::start();
    }

    void stop() override {
        ServiceBase::stop();
        if (diagnosisThread_.joinable()) diagnosisThread_.join();
        if (updateThread_.joinable()) updateThread_.join();
        if (statsThread_.joinable()) statsThread_.join();
        clickhouseClient_->stopBatchWriter();
        clickhouseClient_->disconnect();
        featuresQueue_->close();
        diagnosisQueue_->close();
    }

    void join() override {
        while (running_ && g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        stop();
    }

private:
    void initPrometheusMetrics() {
        auto& registry = MetricsRegistry::instance();

        auto& latency_fam = prometheus::BuildHistogram()
            .Name("turbine_diagnosis_latency_seconds")
            .Help("Acoustic diagnosis processing latency")
            .Register(*getPrometheusRegistry());
        diagnosis_latency_histogram_ = &latency_fam.Add({},
            prometheus::Histogram::BucketBoundaries{
                0.0001, 0.0005, 0.001, 0.0025, 0.005, 0.01, 0.025, 0.05, 0.1});

        auto& match_fam = prometheus::BuildCounter()
            .Name("turbine_pattern_match_total")
            .Help("Pattern matches by cavitation type")
            .Register(*getPrometheusRegistry());
        pattern_match_unknown_ = &match_fam.Add({{"type", "unknown"}});
        pattern_match_cloud_   = &match_fam.Add({{"type", "cloud"}});
        pattern_match_sheet_   = &match_fam.Add({{"type", "sheet"}});
        pattern_match_super_   = &match_fam.Add({{"type", "super"}});
        pattern_match_vortex_  = &match_fam.Add({{"type", "vortex"}});
        pattern_match_other_   = &match_fam.Add({{"type", "other"}});

        auto& unknown_fam = prometheus::BuildCounter()
            .Name("turbine_unknown_pattern_total")
            .Help("Unknown/novel pattern detections")
            .Register(*getPrometheusRegistry());
        unknown_pattern_counter_ = &unknown_fam.Add({});

        auto& cluster_fam = prometheus::BuildGauge()
            .Name("turbine_active_cluster_count")
            .Help("Number of active clusters in K-Means")
            .Register(*getPrometheusRegistry());
        cluster_count_gauge_ = &cluster_fam.Add({});
    }

    void recordPatternMatch(CavitationType type) {
        switch (type) {
            case CavitationType::UNKNOWN:     pattern_match_unknown_->Increment(); break;
            case CavitationType::CLOUD:       pattern_match_cloud_->Increment();   break;
            case CavitationType::SHEET:       pattern_match_sheet_->Increment();   break;
            case CavitationType::SUPER:       pattern_match_super_->Increment();   break;
            case CavitationType::VORTEX:      pattern_match_vortex_->Increment();  break;
            default:                          pattern_match_other_->Increment();   break;
        }
    }

    void diagnosisLoop() {
        IPCMessageFeatures featMsg{};

        while (g_running && running_) {
            size_t processedInBatch = 0;
            const size_t maxBatch = 500;

            while (processedInBatch < maxBatch &&
                   featuresQueue_->pop(featMsg) && g_running) {

                auto t_start = std::chrono::high_resolution_clock::now();

                DiagnosisResult result = diagnosis_->diagnose(featMsg);

                auto t_end = std::chrono::high_resolution_clock::now();
                double latency_s = std::chrono::duration<double>(t_end - t_start).count();
                diagnosis_latency_histogram_->Observe(latency_s);

                recordPatternMatch(result.cavitation_type);
                if (!result.is_known_pattern) {
                    unknown_pattern_counter_->Increment();
                }

                writeToClickHouse(result);

                IPCMessageDiagnosis diagMsg{};
                diagMsg.timestamp = result.timestamp;
                diagMsg.turbine_id = result.turbine_id;
                diagMsg.sensor_id = result.sensor_id;
                diagMsg.cavitation_type = static_cast<uint8_t>(result.cavitation_type);
                diagMsg.diagnosis_status = static_cast<uint8_t>(result.status);
                diagMsg.cluster_label = result.cluster_label;
                diagMsg.is_known_pattern = result.is_known_pattern ? 1 : 0;

                for (size_t i = 0; i < 32 && i < result.embedding.size(); ++i) {
                    diagMsg.feature_embedding[i] = result.embedding[i];
                }
                for (size_t i = 0; i < 4 && i < result.pattern_similarity.size(); ++i) {
                    diagMsg.pattern_similarity[i] = result.pattern_similarity[i];
                }
                for (size_t i = 0; i < 4 && i < result.confidence_scores.size(); ++i) {
                    diagMsg.confidence_scores[i] = result.confidence_scores[i];
                }

                diagMsg.centroid_distance = result.centroid_distance;
                diagMsg.silhouette_score = result.silhouette_score;
                diagMsg.cluster_purity = result.cluster_purity;

                strncpy_safe(diagMsg.cavitation_type_name, result.cavitation_type_name, 32);
                strncpy_safe(diagMsg.expert_note, result.expert_note, 256);

                if (!diagnosisQueue_->push(diagMsg)) {
                    std::lock_guard<std::mutex> lk(statsMutex_);
                    stats_.diagnosisDropped++;
                }

                {
                    std::lock_guard<std::mutex> lk(statsMutex_);
                    stats_.featuresConsumed++;
                    stats_.diagnosisProduced++;
                }
                processedInBatch++;
            }

            cluster_count_gauge_->Set(static_cast<double>(diagnosis_->clustering().clusterCount()));

            if (processedInBatch == 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }
    }

    void periodicUpdateLoop() {
        while (g_running && running_) {
            uint32_t interval_s = config_.diagnosis.update_interval_s;
            for (uint32_t i = 0; i < interval_s * 10 && g_running && running_; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (!g_running || !running_) break;

            try {
                diagnosis_->periodicUpdate();
                syncPatternLibraryToDB();

                std::lock_guard<std::mutex> lk(statsMutex_);
                stats_.periodicUpdates++;
            } catch (const std::exception& e) {
                std::cerr << "[" << name_ << "] periodicUpdate error: " << e.what() << std::endl;
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

            auto featStats = featuresQueue_->getStats();
            auto diagStats = diagnosisQueue_->getStats();

            std::lock_guard<std::mutex> lk(statsMutex_);
            std::cout << "\r[acoustic_diagnosis] "
                      << "in=" << stats_.featuresConsumed
                      << " | out=" << stats_.diagnosisProduced
                      << " | feat_q=" << featStats.currentSize
                      << " | diag_q=" << diagStats.currentSize
                      << " | drop=" << stats_.diagnosisDropped
                      << " | upd=" << stats_.periodicUpdates
                      << std::flush;
        }
    }

    void writeToClickHouse(const DiagnosisResult& result) {
        if (!clickhouseClient_) return;

        std::ostringstream query;
        query << "INSERT INTO diagnosis_results (timestamp, turbine_id, sensor_id, "
              << "cavitation_type, diagnosis_status, cluster_label, is_known_pattern, "
              << "embedding, pattern_similarity, confidence_scores, centroid_distance, "
              << "silhouette_score, cluster_purity, cavitation_type_name, expert_note, "
              << "analysis_latency_us) VALUES (";

        query << result.timestamp << ","
              << static_cast<uint32_t>(result.turbine_id) << ","
              << static_cast<uint32_t>(result.sensor_id) << ","
              << static_cast<uint32_t>(result.cavitation_type) << ","
              << static_cast<uint32_t>(result.status) << ","
              << static_cast<uint32_t>(result.cluster_label) << ","
              << (result.is_known_pattern ? 1 : 0) << ",";

        query << "[";
        for (size_t i = 0; i < result.embedding.size(); ++i) {
            if (i > 0) query << ",";
            query << result.embedding[i];
        }
        query << "],";

        query << "[";
        for (size_t i = 0; i < result.pattern_similarity.size(); ++i) {
            if (i > 0) query << ",";
            query << result.pattern_similarity[i];
        }
        query << "],";

        query << "[";
        for (size_t i = 0; i < result.confidence_scores.size(); ++i) {
            if (i > 0) query << ",";
            query << result.confidence_scores[i];
        }
        query << "],";

        query << result.centroid_distance << ","
              << result.silhouette_score << ","
              << result.cluster_purity << ","
              << "'" << escapeSQL(result.cavitation_type_name) << "',"
              << "'" << escapeSQL(result.expert_note) << "',"
              << result.analysis_latency_us << ")";

        try {
            getQueryMutex().lock();
            std::string q = query.str();
            getQueryMutex().unlock();
        } catch (...) {}
    }

    void syncPatternLibraryToDB() {
        if (!clickhouseClient_) return;

        const auto& patterns = diagnosis_->patternLibrary().getPatterns();
        for (const auto& [type, pattern] : patterns) {
            std::ostringstream query;
            query << "INSERT INTO acoustic_patterns (timestamp, cavitation_type, pattern_name, "
                  << "description, embedding, centroid, sample_count, intra_cluster_variance, "
                  << "silhouette_score, is_verified_by_expert, expert_note, last_updated) VALUES (";

            query << pattern.timestamp << ","
                  << static_cast<uint32_t>(pattern.cavitation_type) << ","
                  << "'" << escapeSQL(pattern.pattern_name) << "',"
                  << "'" << escapeSQL(pattern.description) << "',[";

            for (size_t i = 0; i < pattern.embedding.size(); ++i) {
                if (i > 0) query << ",";
                query << pattern.embedding[i];
            }
            query << "],[";

            for (size_t i = 0; i < pattern.centroid.size(); ++i) {
                if (i > 0) query << ",";
                query << pattern.centroid[i];
            }
            query << "],";

            query << pattern.sample_count << ","
                  << pattern.intra_cluster_variance << ","
                  << pattern.silhouette_score << ","
                  << (pattern.is_verified_by_expert ? 1 : 0) << ","
                  << "'" << escapeSQL(pattern.expert_note) << "',"
                  << pattern.last_updated << ")";
        }
    }

    static std::string escapeSQL(const std::string& s) {
        std::string result;
        result.reserve(s.size() + 8);
        for (char c : s) {
            if (c == '\'') result += "''";
            else if (c == '\\') result += "\\\\";
            else result += c;
        }
        return result;
    }

    static std::shared_ptr<prometheus::Registry> getPrometheusRegistry() {
        static auto registry = std::make_shared<prometheus::Registry>();
        return registry;
    }

    static std::mutex& getQueryMutex() {
        static std::mutex mtx;
        return mtx;
    }

    Config config_;
    std::unique_ptr<AcousticDiagnosisFacade> diagnosis_;
    std::unique_ptr<ClickHouseClient> clickhouseClient_;
    std::unique_ptr<SharedMemorySPSC<IPCMessageFeatures>> featuresQueue_;
    std::unique_ptr<SharedMemorySPSC<IPCMessageDiagnosis>> diagnosisQueue_;

    std::thread diagnosisThread_;
    std::thread updateThread_;
    std::thread statsThread_;

    prometheus::Histogram* diagnosis_latency_histogram_;
    prometheus::Counter* pattern_match_unknown_;
    prometheus::Counter* pattern_match_cloud_;
    prometheus::Counter* pattern_match_sheet_;
    prometheus::Counter* pattern_match_super_;
    prometheus::Counter* pattern_match_vortex_;
    prometheus::Counter* pattern_match_other_;
    prometheus::Counter* unknown_pattern_counter_;
    prometheus::Gauge* cluster_count_gauge_;

    struct ServiceStats {
        uint64_t featuresConsumed = 0;
        uint64_t diagnosisProduced = 0;
        uint64_t diagnosisDropped = 0;
        uint64_t periodicUpdates = 0;
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
        std::cerr << "[acoustic_diagnosis] Warning: Failed to load config, using defaults" << std::endl;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const Config& cfg = getConfig();

    auto service = std::make_unique<AcousticDiagnosisService>();
    if (!service->init(cfg)) {
        std::cerr << "[acoustic_diagnosis] Init failed" << std::endl;
        return 1;
    }

    if (!service->start()) {
        std::cerr << "[acoustic_diagnosis] Start failed" << std::endl;
        return 1;
    }

    service->join();
    std::cout << std::endl << "[acoustic_diagnosis] Shutdown complete" << std::endl;
    return 0;
}
