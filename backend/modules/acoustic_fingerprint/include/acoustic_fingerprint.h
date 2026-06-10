#pragma once

#include "data_structures.h"
#include "config.h"
#include <vector>
#include <array>
#include <string>
#include <mutex>
#include <unordered_map>
#include <optional>
#include <random>
#include <memory>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <cstring>

namespace acoustic_fp {

using Embedding = std::array<float, 32>;
using FeatureVector = std::array<float, 32>;
using Spectrum = std::vector<float>;

constexpr size_t FEATURE_DIM  = 32;
constexpr size_t EMBEDDING_DIM = 32;
constexpr size_t FFT_SIZE     = 1024;
constexpr size_t N_FFT_BINS   = 512;

struct GPUAccelConfig {
    bool   enabled          = true;
    size_t batch_size       = 128;
    size_t threads_per_block = 256;
    bool   use_tensor_cores  = true;
    size_t denoise_iterations = 2;
};

struct DenoiseConfig {
    size_t median_kernel   = 3;
    float  adaptive_bw_min = 1.1f;
    float  adaptive_bw_max = 3.9f;
    float  softmax_temp    = 0.05f;
    float  pull_weight     = 0.50f;
    float  impulse_thresh  = 6.5f;
    float  impulse_pct     = 0.12f;
};

struct Triplet {
    Embedding anchor;
    Embedding positive;
    Embedding negative;
    float anchor_pos_dist;
    float anchor_neg_dist;
};

class GPUFeatureExtractor {
public:
    GPUFeatureExtractor() = default;
    explicit GPUFeatureExtractor(const GPUAccelConfig& cfg) : cfg_(cfg) {}

    void setConfig(const GPUAccelConfig& cfg) { cfg_ = cfg; }
    const GPUAccelConfig& config() const { return cfg_; }

    bool isAvailable() const { return available_; }
    void setAvailable(bool a) { available_ = a; }

    void extractBatch(const std::vector<Spectrum>& spectra,
                      std::vector<FeatureVector>& features) const;

    FeatureVector extract(const Spectrum& spectrum) const;

    std::vector<Spectrum> denoiseBatch(const std::vector<Spectrum>& noisy,
                                       const std::vector<Embedding>& centroids,
                                       size_t iterations = 2) const;

    Spectrum denoise(const Spectrum& noisy,
                     const std::vector<Embedding>& centroids,
                     size_t iterations = 2) const;

private:
    GPUAccelConfig cfg_;
    bool available_{true};

    static FeatureVector cpuExtract(const Spectrum& spectrum);
    static Spectrum cpuDenoiseIter(const Spectrum& x,
                                   const std::vector<Embedding>& centroids,
                                   const DenoiseConfig& dcfg);
};

class EmbeddingNetwork {
public:
    EmbeddingNetwork() = default;
    ~EmbeddingNetwork() = default;

    bool loadWeights(const std::string& path);

    Embedding infer(const FeatureVector& feature_vector);

    bool isLoaded() const { return weights_loaded_; }

    void setDefaultWeights();

private:
    static void relu(std::vector<float>& x);
    static void batchNorm(std::vector<float>& x,
                          const std::vector<float>& gamma,
                          const std::vector<float>& beta,
                          const std::vector<float>& running_mean,
                          const std::vector<float>& running_var,
                          float eps = 1e-5f);
    static void linear(const std::vector<float>& input,
                       const std::vector<std::vector<float>>& weight,
                       const std::vector<float>& bias,
                       std::vector<float>& output);
    static void l2Normalize(Embedding& embedding);

    bool weights_loaded_ = false;

    std::vector<std::vector<float>> w1_;
    std::vector<float> b1_;
    std::vector<float> bn_gamma_;
    std::vector<float> bn_beta_;
    std::vector<float> bn_running_mean_;
    std::vector<float> bn_running_var_;
    std::vector<std::vector<float>> w2_;
    std::vector<float> b2_;
    std::vector<std::vector<float>> w3_;
    std::vector<float> b3_;
};

class TripletMiner {
public:
    explicit TripletMiner(float margin = 0.3f) : margin_(margin) {}
    ~TripletMiner() = default;

    std::vector<Triplet> hardNegativeMining(
        const Embedding& anchor_emb,
        const std::vector<Embedding>& pos_pool,
        const std::vector<Embedding>& neg_pool);

    void setMargin(float margin) { margin_ = margin; }

private:
    static float euclideanDistance(const Embedding& a, const Embedding& b);

    float margin_;
};

struct Cluster {
    uint8_t label;
    Embedding centroid;
    uint64_t sample_count;
    float intra_cluster_variance;
    std::vector<Embedding> samples;
};

class OnlineKMeansClustering {
public:
    explicit OnlineKMeansClustering(
        uint8_t max_clusters = 10,
        float merge_threshold = 0.5f,
        uint32_t min_samples = 5);
    ~OnlineKMeansClustering() = default;

    uint8_t assign(const Embedding& embedding);

    void update(const Embedding& embedding, uint8_t cluster_label);

    void clusterMerge();

    float silhouetteCoefficient(
        const std::vector<Embedding>& embeddings,
        const std::vector<uint8_t>& labels);

    float distanceToCentroid(const Embedding& embedding, uint8_t cluster_label);

    const Cluster& getCluster(uint8_t label) const;

    uint8_t clusterCount() const { return static_cast<uint8_t>(clusters_.size()); }

    const std::vector<Cluster>& getClusters() const { return clusters_; }

    void recomputeGlobalCentroids();

    void clear();

private:
    uint8_t findNearestCluster(const Embedding& embedding, float& min_dist);
    static float cosineSimilarity(const Embedding& a, const Embedding& b);
    static float euclideanDistance(const Embedding& a, const Embedding& b);

    uint8_t max_clusters_;
    float merge_threshold_;
    uint32_t min_samples_;
    std::vector<Cluster> clusters_;
    mutable std::mutex mutex_;
    std::mt19937 rng_;
    bool kmeanspp_initialized_ = false;
    std::vector<Embedding> seed_buffer_;
};

struct PatternMatchResult {
    CavitationType cavitation_type;
    std::string pattern_name;
    float similarity;
    float centroid_distance;
    bool is_known;
};

class PatternLibrary {
public:
    PatternLibrary();
    ~PatternLibrary() = default;

    void initializeDefaultPatterns();

    PatternMatchResult match(const Embedding& embedding);

    std::vector<PatternMatchResult> matchTopN(const Embedding& embedding, size_t n = 4);

    bool detectNovelty(const Embedding& embedding, uint8_t cluster_label);

    bool addPattern(
        CavitationType cavitation_type,
        const std::string& name,
        const std::vector<Embedding>& samples,
        bool is_expert_verified = false,
        const std::string& expert_note = "");

    const AcousticPattern& getPattern(CavitationType type) const;

    bool hasPattern(CavitationType type) const;

    void updatePatternCentroid(CavitationType type, const Embedding& new_centroid);

    const std::unordered_map<CavitationType, AcousticPattern>& getPatterns() const {
        return patterns_;
    }

    void setSimilarityThreshold(float t) { similarity_threshold_ = t; }
    void setNoveltyThreshold(float t) { novelty_threshold_ = t; }

    static std::string cavitationTypeDescription(CavitationType type);

private:
    static float cosineSimilarity(const Embedding& a, const Embedding& b);
    static Embedding computeCentroid(const std::vector<Embedding>& samples);
    static float computeIntraVariance(
        const std::vector<Embedding>& samples,
        const Embedding& centroid);
    static void strncpy_safe(char* dest, const std::string& src, size_t max_len);

    std::unordered_map<CavitationType, AcousticPattern> patterns_;
    float similarity_threshold_ = 0.75f;
    float novelty_threshold_ = 0.15f;
    mutable std::mutex mutex_;
};

class AcousticFingerprintFacade {
public:
    using DiagnosisCallback = std::function<void(const DiagnosisResult&)>;

    AcousticFingerprintFacade();
    ~AcousticFingerprintFacade();

    bool init(const Config::DiagnosisConfig& config);

    DiagnosisResult diagnose(const IPCMessageFeatures& feature_msg);

    void periodicUpdate();

    void asyncDiagnose(const IPCMessageFeatures& feature_msg);

    void setCallback(DiagnosisCallback cb) { cb_ = std::move(cb); }

    void setDenoiseEnabled(bool enabled) { denoise_enabled_ = enabled; }
    bool denoiseEnabled() const { return denoise_enabled_; }

    void setGPUAccelerated(bool enabled) { gpu_extractor_.setAvailable(enabled); gpu_enabled_ = enabled; }
    bool gpuAccelerated() const { return gpu_enabled_; }

    PatternLibrary& patternLibrary() { return pattern_library_; }
    OnlineKMeansClustering& clustering() { return clustering_; }
    EmbeddingNetwork& network() { return network_; }
    GPUFeatureExtractor& gpuExtractor() { return gpu_extractor_; }

    uint64_t getTotalDiagnosed() const { return total_diagnosed_.load(); }
    uint64_t getKnownMatches() const { return known_matches_.load(); }
    uint64_t getUnknownPatterns() const { return unknown_patterns_.load(); }

    void start();
    void stop();

private:
    Embedding extractEmbedding(const IPCMessageFeatures& feature_msg);
    PatternMatchResult matchKnown(const Embedding& embedding);
    uint8_t clusterUnknown(const Embedding& embedding);
    std::vector<float> computeConfidenceScores(
        const Embedding& embedding,
        const PatternMatchResult& match_result,
        uint8_t cluster_label);

    static FeatureVector buildFeatureVector(const IPCMessageFeatures& msg);
    static std::string cavitationTypeName(CavitationType type);

    void workerLoop();

    Config::DiagnosisConfig config_;
    EmbeddingNetwork network_;
    TripletMiner triplet_miner_;
    OnlineKMeansClustering clustering_;
    PatternLibrary pattern_library_;
    GPUFeatureExtractor gpu_extractor_;
    DenoiseConfig denoise_cfg_;

    std::atomic<bool> denoise_enabled_{true};
    std::atomic<bool> gpu_enabled_{true};

    std::atomic<uint64_t> total_diagnosed_{0};
    std::atomic<uint64_t> known_matches_{0};
    std::atomic<uint64_t> unknown_patterns_{0};
    uint64_t last_update_ts_ = 0;

    DiagnosisCallback cb_;

    std::atomic<bool> running_{false};
    std::unique_ptr<std::thread> worker_;
    std::mutex cv_mutex_;
    std::condition_variable cv_;
    std::deque<IPCMessageFeatures> queue_;
    mutable std::mutex queue_mutex_;
};

}
