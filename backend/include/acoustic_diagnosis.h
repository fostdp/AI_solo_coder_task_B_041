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

namespace turbine_monitor {

using Embedding = std::array<float, 32>;
using FeatureVector = std::array<float, 32>;

struct Triplet {
    Embedding anchor;
    Embedding positive;
    Embedding negative;
    float anchor_pos_dist;
    float anchor_neg_dist;
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

class AcousticDiagnosisFacade {
public:
    AcousticDiagnosisFacade();
    ~AcousticDiagnosisFacade() = default;

    bool init(const Config::DiagnosisConfig& config);

    DiagnosisResult diagnose(const IPCMessageFeatures& feature_msg);

    void periodicUpdate();

    PatternLibrary& patternLibrary() { return pattern_library_; }
    OnlineKMeansClustering& clustering() { return clustering_; }
    EmbeddingNetwork& network() { return network_; }

    uint64_t getTotalDiagnosed() const { return total_diagnosed_; }
    uint64_t getKnownMatches() const { return known_matches_; }
    uint64_t getUnknownPatterns() const { return unknown_patterns_; }

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

    Config::DiagnosisConfig config_;
    EmbeddingNetwork network_;
    TripletMiner triplet_miner_;
    OnlineKMeansClustering clustering_;
    PatternLibrary pattern_library_;

    uint64_t total_diagnosed_ = 0;
    uint64_t known_matches_ = 0;
    uint64_t unknown_patterns_ = 0;
    uint64_t last_update_ts_ = 0;

    mutable std::mutex mutex_;
};

}
