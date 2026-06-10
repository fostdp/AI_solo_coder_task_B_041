#include "acoustic_diagnosis.h"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <cstring>

namespace turbine_monitor {

static constexpr float EPS = 1e-8f;

static Embedding createCloudCentroid() {
    Embedding e{};
    e[0] = 0.85f; e[1] = 0.72f; e[2] = 0.91f; e[3] = 0.68f;
    e[4] = 0.95f; e[5] = 0.78f; e[6] = 0.88f; e[7] = 0.65f;
    e[8] = 0.72f; e[9] = 0.58f; e[10] = 0.81f; e[11] = 0.74f;
    e[12] = 0.67f; e[13] = 0.93f; e[14] = 0.82f; e[15] = 0.76f;
    e[16] = 0.55f; e[17] = 0.89f; e[18] = 0.71f; e[19] = 0.94f;
    e[20] = 0.63f; e[21] = 0.77f; e[22] = 0.85f; e[23] = 0.69f;
    e[24] = 0.92f; e[25] = 0.73f; e[26] = 0.80f; e[27] = 0.87f;
    e[28] = 0.66f; e[29] = 0.79f; e[30] = 0.70f; e[31] = 0.90f;
    float norm = 0.0f;
    for (float v : e) norm += v * v;
    norm = std::sqrt(norm);
    for (float& v : e) v /= norm;
    return e;
}

static Embedding createSheetCentroid() {
    Embedding e{};
    e[0] = 0.55f; e[1] = 0.82f; e[2] = 0.68f; e[3] = 0.93f;
    e[4] = 0.71f; e[5] = 0.88f; e[6] = 0.62f; e[7] = 0.95f;
    e[8] = 0.85f; e[9] = 0.76f; e[10] = 0.90f; e[11] = 0.69f;
    e[12] = 0.78f; e[13] = 0.58f; e[14] = 0.92f; e[15] = 0.73f;
    e[16] = 0.87f; e[17] = 0.64f; e[18] = 0.81f; e[19] = 0.70f;
    e[20] = 0.94f; e[21] = 0.66f; e[22] = 0.75f; e[23] = 0.89f;
    e[24] = 0.57f; e[25] = 0.91f; e[26] = 0.79f; e[27] = 0.61f;
    e[28] = 0.86f; e[29] = 0.72f; e[30] = 0.96f; e[31] = 0.67f;
    float norm = 0.0f;
    for (float v : e) norm += v * v;
    norm = std::sqrt(norm);
    for (float& v : e) v /= norm;
    return e;
}

static Embedding createSuperCentroid() {
    Embedding e{};
    e[0] = 0.92f; e[1] = 0.55f; e[2] = 0.86f; e[3] = 0.48f;
    e[4] = 0.97f; e[5] = 0.51f; e[6] = 0.78f; e[7] = 0.62f;
    e[8] = 0.45f; e[9] = 0.91f; e[10] = 0.58f; e[11] = 0.89f;
    e[12] = 0.53f; e[13] = 0.84f; e[14] = 0.66f; e[15] = 0.95f;
    e[16] = 0.70f; e[17] = 0.57f; e[18] = 0.93f; e[19] = 0.49f;
    e[20] = 0.82f; e[21] = 0.61f; e[22] = 0.96f; e[23] = 0.54f;
    e[24] = 0.75f; e[25] = 0.88f; e[26] = 0.50f; e[27] = 0.80f;
    e[28] = 0.63f; e[29] = 0.94f; e[30] = 0.56f; e[31] = 0.87f;
    float norm = 0.0f;
    for (float v : e) norm += v * v;
    norm = std::sqrt(norm);
    for (float& v : e) v /= norm;
    return e;
}

static Embedding createVortexCentroid() {
    Embedding e{};
    e[0] = 0.62f; e[1] = 0.90f; e[2] = 0.54f; e[3] = 0.86f;
    e[4] = 0.77f; e[5] = 0.65f; e[6] = 0.93f; e[7] = 0.58f;
    e[8] = 0.81f; e[9] = 0.72f; e[10] = 0.60f; e[11] = 0.94f;
    e[12] = 0.56f; e[13] = 0.79f; e[14] = 0.68f; e[15] = 0.85f;
    e[16] = 0.92f; e[17] = 0.61f; e[18] = 0.74f; e[19] = 0.88f;
    e[20] = 0.52f; e[21] = 0.83f; e[22] = 0.59f; e[23] = 0.91f;
    e[24] = 0.70f; e[25] = 0.64f; e[26] = 0.87f; e[27] = 0.55f;
    e[28] = 0.78f; e[29] = 0.69f; e[30] = 0.95f; e[31] = 0.63f;
    float norm = 0.0f;
    for (float v : e) norm += v * v;
    norm = std::sqrt(norm);
    for (float& v : e) v /= norm;
    return e;
}

static std::vector<Embedding> generatePatternSamples(const Embedding& centroid, size_t n, float noise_std = 0.05f) {
    std::vector<Embedding> samples;
    samples.reserve(n);
    std::mt19937 rng(42);
    std::normal_distribution<float> noise(0.0f, noise_std);

    for (size_t i = 0; i < n; ++i) {
        Embedding s;
        for (size_t j = 0; j < 32; ++j) {
            s[j] = centroid[j] + noise(rng);
        }
        float norm = 0.0f;
        for (float v : s) norm += v * v;
        norm = std::sqrt(norm);
        for (float& v : s) v /= norm;
        samples.push_back(s);
    }
    return samples;
}

static void initLayer(std::vector<std::vector<float>>& w,
                      std::vector<float>& b,
                      size_t in_dim, size_t out_dim) {
    w.resize(out_dim, std::vector<float>(in_dim));
    b.resize(out_dim, 0.0f);

    float scale = std::sqrt(2.0f / static_cast<float>(in_dim));
    std::mt19937 rng(12345);
    std::normal_distribution<float> dist(0.0f, scale);

    for (size_t i = 0; i < out_dim; ++i) {
        for (size_t j = 0; j < in_dim; ++j) {
            w[i][j] = dist(rng);
        }
    }
}

void EmbeddingNetwork::setDefaultWeights() {
    initLayer(w1_, b1_, 32, 64);
    bn_gamma_.resize(64, 1.0f);
    bn_beta_.resize(64, 0.0f);
    bn_running_mean_.resize(64, 0.0f);
    bn_running_var_.resize(64, 1.0f);
    initLayer(w2_, b2_, 64, 48);
    initLayer(w3_, b3_, 48, 32);
    weights_loaded_ = true;
}

bool EmbeddingNetwork::loadWeights(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        setDefaultWeights();
        return false;
    }

    try {
        std::string content((std::istreambuf_iterator<char>(f)),
                             std::istreambuf_iterator<char>());
        f.close();
    } catch (...) {
        f.close();
    }

    setDefaultWeights();
    return true;
}

void EmbeddingNetwork::relu(std::vector<float>& x) {
    for (float& v : x) {
        v = v > 0.0f ? v : 0.0f;
    }
}

void EmbeddingNetwork::batchNorm(std::vector<float>& x,
                                  const std::vector<float>& gamma,
                                  const std::vector<float>& beta,
                                  const std::vector<float>& running_mean,
                                  const std::vector<float>& running_var,
                                  float eps) {
    size_t n = x.size();
    for (size_t i = 0; i < n; ++i) {
        float inv_std = 1.0f / std::sqrt(running_var[i] + eps);
        x[i] = gamma[i] * (x[i] - running_mean[i]) * inv_std + beta[i];
    }
}

void EmbeddingNetwork::linear(const std::vector<float>& input,
                               const std::vector<std::vector<float>>& weight,
                               const std::vector<float>& bias,
                               std::vector<float>& output) {
    size_t out_dim = weight.size();
    size_t in_dim = weight[0].size();
    output.resize(out_dim);

    for (size_t i = 0; i < out_dim; ++i) {
        float sum = bias[i];
        const auto& row = weight[i];
        for (size_t j = 0; j < in_dim; ++j) {
            sum += row[j] * input[j];
        }
        output[i] = sum;
    }
}

void EmbeddingNetwork::l2Normalize(Embedding& embedding) {
    float sum_sq = 0.0f;
    for (float v : embedding) sum_sq += v * v;
    float inv_norm = 1.0f / (std::sqrt(sum_sq) + EPS);
    for (float& v : embedding) v *= inv_norm;
}

Embedding EmbeddingNetwork::infer(const FeatureVector& feature_vector) {
    if (!weights_loaded_) {
        setDefaultWeights();
    }

    std::vector<float> h1(feature_vector.begin(), feature_vector.end());
    std::vector<float> h1_out;
    linear(h1, w1_, b1_, h1_out);
    relu(h1_out);
    batchNorm(h1_out, bn_gamma_, bn_beta_, bn_running_mean_, bn_running_var_);

    std::vector<float> h2_out;
    linear(h1_out, w2_, b2_, h2_out);
    relu(h2_out);

    std::vector<float> h3_out;
    linear(h2_out, w3_, b3_, h3_out);

    Embedding embedding;
    std::copy_n(h3_out.begin(), 32, embedding.begin());
    l2Normalize(embedding);

    return embedding;
}

float TripletMiner::euclideanDistance(const Embedding& a, const Embedding& b) {
    float sum_sq = 0.0f;
    for (size_t i = 0; i < 32; ++i) {
        float d = a[i] - b[i];
        sum_sq += d * d;
    }
    return std::sqrt(sum_sq);
}

std::vector<Triplet> TripletMiner::hardNegativeMining(
    const Embedding& anchor_emb,
    const std::vector<Embedding>& pos_pool,
    const std::vector<Embedding>& neg_pool) {

    std::vector<Triplet> result;
    if (pos_pool.empty() || neg_pool.empty()) return result;

    for (const auto& pos : pos_pool) {
        float d_ap = euclideanDistance(anchor_emb, pos);

        std::vector<std::pair<float, Embedding>> semi_hard_negatives;
        for (const auto& neg : neg_pool) {
            float d_an = euclideanDistance(anchor_emb, neg);
            if (d_ap < d_an && d_an < d_ap + margin_) {
                semi_hard_negatives.emplace_back(d_an, neg);
            }
        }

        if (!semi_hard_negatives.empty()) {
            std::sort(semi_hard_negatives.begin(), semi_hard_negatives.end(),
                      [](const auto& a, const auto& b) { return a.first < b.first; });
            const auto& best = semi_hard_negatives.front();
            result.push_back({anchor_emb, pos, best.second, d_ap, best.first});
        } else {
            std::vector<std::pair<float, Embedding>> all_dists;
            all_dists.reserve(neg_pool.size());
            for (const auto& neg : neg_pool) {
                float d_an = euclideanDistance(anchor_emb, neg);
                all_dists.emplace_back(d_an, neg);
            }
            std::sort(all_dists.begin(), all_dists.end(),
                      [](const auto& a, const auto& b) { return a.first < b.first; });
            const auto& hardest = all_dists.front();
            result.push_back({anchor_emb, pos, hardest.second, d_ap, hardest.first});
        }
    }

    return result;
}

OnlineKMeansClustering::OnlineKMeansClustering(
    uint8_t max_clusters,
    float merge_threshold,
    uint32_t min_samples)
    : max_clusters_(max_clusters),
      merge_threshold_(merge_threshold),
      min_samples_(min_samples),
      rng_(std::random_device{}()) {}

float OnlineKMeansClustering::cosineSimilarity(const Embedding& a, const Embedding& b) {
    float dot = 0.0f, norm_a = 0.0f, norm_b = 0.0f;
    for (size_t i = 0; i < 32; ++i) {
        dot += a[i] * b[i];
        norm_a += a[i] * a[i];
        norm_b += b[i] * b[i];
    }
    float denom = std::sqrt(norm_a) * std::sqrt(norm_b) + EPS;
    return dot / denom;
}

float OnlineKMeansClustering::euclideanDistance(const Embedding& a, const Embedding& b) {
    float sum_sq = 0.0f;
    for (size_t i = 0; i < 32; ++i) {
        float d = a[i] - b[i];
        sum_sq += d * d;
    }
    return std::sqrt(sum_sq);
}

uint8_t OnlineKMeansClustering::findNearestCluster(const Embedding& embedding, float& min_dist) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (clusters_.empty()) {
        min_dist = std::numeric_limits<float>::max();
        return 0xFF;
    }

    uint8_t best_label = clusters_[0].label;
    min_dist = euclideanDistance(embedding, clusters_[0].centroid);

    for (size_t i = 1; i < clusters_.size(); ++i) {
        float d = euclideanDistance(embedding, clusters_[i].centroid);
        if (d < min_dist) {
            min_dist = d;
            best_label = clusters_[i].label;
        }
    }
    return best_label;
}

uint8_t OnlineKMeansClustering::assign(const Embedding& embedding) {
    std::lock_guard<std::mutex> lk(mutex_);

    if (clusters_.empty()) {
        if (seed_buffer_.size() < max_clusters_) {
            seed_buffer_.push_back(embedding);
        }

        if (seed_buffer_.size() >= static_cast<size_t>(max_clusters_) ||
            seed_buffer_.size() >= min_samples_) {
            uint8_t label = 0;
            for (const auto& seed : seed_buffer_) {
                Cluster c;
                c.label = label++;
                c.centroid = seed;
                c.sample_count = 1;
                c.intra_cluster_variance = 0.0f;
                c.samples.push_back(seed);
                clusters_.push_back(c);
            }
            seed_buffer_.clear();
            kmeanspp_initialized_ = true;
        }
        return 0;
    }

    uint8_t best_idx = 0;
    float min_dist = euclideanDistance(embedding, clusters_[0].centroid);

    for (size_t i = 1; i < clusters_.size(); ++i) {
        float d = euclideanDistance(embedding, clusters_[i].centroid);
        if (d < min_dist) {
            min_dist = d;
            best_idx = static_cast<uint8_t>(i);
        }
    }

    return clusters_[best_idx].label;
}

void OnlineKMeansClustering::update(const Embedding& embedding, uint8_t cluster_label) {
    std::lock_guard<std::mutex> lk(mutex_);

    int target_idx = -1;
    for (size_t i = 0; i < clusters_.size(); ++i) {
        if (clusters_[i].label == cluster_label) {
            target_idx = static_cast<int>(i);
            break;
        }
    }

    if (target_idx < 0) {
        if (clusters_.size() < max_clusters_) {
            Cluster c;
            c.label = static_cast<uint8_t>(clusters_.size());
            c.centroid = embedding;
            c.sample_count = 1;
            c.intra_cluster_variance = 0.0f;
            c.samples.push_back(embedding);
            clusters_.push_back(c);
        }
        return;
    }

    Cluster& c = clusters_[target_idx];
    c.sample_count++;
    float alpha = 1.0f / static_cast<float>(c.sample_count);
    for (size_t i = 0; i < 32; ++i) {
        c.centroid[i] = (1.0f - alpha) * c.centroid[i] + alpha * embedding[i];
    }

    if (c.samples.size() < 200) {
        c.samples.push_back(embedding);
    } else {
        size_t idx = std::uniform_int_distribution<size_t>(0, c.samples.size() - 1)(rng_);
        c.samples[idx] = embedding;
    }
}

void OnlineKMeansClustering::clusterMerge() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (clusters_.size() < 2) return;

    std::vector<bool> merged(clusters_.size(), false);
    bool any_merged = true;

    while (any_merged) {
        any_merged = false;
        for (size_t i = 0; i < clusters_.size() && !any_merged; ++i) {
            if (merged[i]) continue;
            for (size_t j = i + 1; j < clusters_.size() && !any_merged; ++j) {
                if (merged[j]) continue;
                float d = euclideanDistance(clusters_[i].centroid, clusters_[j].centroid);
                if (d < merge_threshold_) {
                    uint64_t n_i = clusters_[i].sample_count;
                    uint64_t n_j = clusters_[j].sample_count;
                    uint64_t n_total = n_i + n_j;
                    float w_i = static_cast<float>(n_i) / static_cast<float>(n_total);
                    float w_j = static_cast<float>(n_j) / static_cast<float>(n_total);

                    for (size_t k = 0; k < 32; ++k) {
                        clusters_[i].centroid[k] =
                            w_i * clusters_[i].centroid[k] + w_j * clusters_[j].centroid[k];
                    }
                    clusters_[i].sample_count = n_total;

                    for (const auto& s : clusters_[j].samples) {
                        if (clusters_[i].samples.size() < 200) {
                            clusters_[i].samples.push_back(s);
                        }
                    }

                    merged[j] = true;
                    any_merged = true;
                }
            }
        }
    }

    std::vector<Cluster> new_clusters;
    for (size_t i = 0; i < clusters_.size(); ++i) {
        if (!merged[i] && clusters_[i].sample_count >= min_samples_) {
            new_clusters.push_back(clusters_[i]);
        }
    }

    for (size_t i = 0; i < new_clusters.size(); ++i) {
        new_clusters[i].label = static_cast<uint8_t>(i);
    }

    clusters_ = std::move(new_clusters);
}

float OnlineKMeansClustering::silhouetteCoefficient(
    const std::vector<Embedding>& embeddings,
    const std::vector<uint8_t>& labels) {

    if (embeddings.size() < 2) return 0.0f;

    float total_sil = 0.0f;
    size_t count = 0;

    for (size_t i = 0; i < embeddings.size(); ++i) {
        const auto& x = embeddings[i];
        uint8_t label_i = labels[i];

        float a_sum = 0.0f;
        size_t a_count = 0;
        std::unordered_map<uint8_t, std::pair<float, size_t>> cluster_dists;

        for (size_t j = 0; j < embeddings.size(); ++j) {
            if (i == j) continue;
            float d = euclideanDistance(x, embeddings[j]);
            if (labels[j] == label_i) {
                a_sum += d;
                a_count++;
            } else {
                cluster_dists[labels[j]].first += d;
                cluster_dists[labels[j]].second++;
            }
        }

        float a = (a_count > 0) ? a_sum / static_cast<float>(a_count) : 0.0f;

        float b = std::numeric_limits<float>::max();
        for (const auto& [lbl, pair] : cluster_dists) {
            if (pair.second > 0) {
                float avg = pair.first / static_cast<float>(pair.second);
                if (avg < b) b = avg;
            }
        }

        if (a_count > 0 && cluster_dists.size() > 0) {
            float max_ab = std::max(a, b);
            float sil = (max_ab > EPS) ? (b - a) / max_ab : 0.0f;
            total_sil += sil;
            count++;
        }
    }

    return (count > 0) ? total_sil / static_cast<float>(count) : 0.0f;
}

float OnlineKMeansClustering::distanceToCentroid(const Embedding& embedding, uint8_t cluster_label) {
    std::lock_guard<std::mutex> lk(mutex_);
    for (const auto& c : clusters_) {
        if (c.label == cluster_label) {
            return euclideanDistance(embedding, c.centroid);
        }
    }
    return std::numeric_limits<float>::max();
}

const Cluster& OnlineKMeansClustering::getCluster(uint8_t label) const {
    std::lock_guard<std::mutex> lk(mutex_);
    for (const auto& c : clusters_) {
        if (c.label == label) return c;
    }
    static Cluster empty{};
    return empty;
}

void OnlineKMeansClustering::recomputeGlobalCentroids() {
    std::lock_guard<std::mutex> lk(mutex_);
    for (auto& c : clusters_) {
        if (c.samples.empty()) continue;
        Embedding new_centroid{};
        for (const auto& s : c.samples) {
            for (size_t i = 0; i < 32; ++i) {
                new_centroid[i] += s[i];
            }
        }
        float inv_n = 1.0f / static_cast<float>(c.samples.size());
        for (size_t i = 0; i < 32; ++i) {
            new_centroid[i] *= inv_n;
        }
        c.centroid = new_centroid;
    }
}

void OnlineKMeansClustering::clear() {
    std::lock_guard<std::mutex> lk(mutex_);
    clusters_.clear();
    seed_buffer_.clear();
    kmeanspp_initialized_ = false;
}

void PatternLibrary::strncpy_safe(char* dest, const std::string& src, size_t max_len) {
    if (!dest || max_len == 0) return;
    size_t copy_len = std::min(src.size(), max_len - 1);
    std::memcpy(dest, src.c_str(), copy_len);
    dest[copy_len] = '\0';
}

float PatternLibrary::cosineSimilarity(const Embedding& a, const Embedding& b) {
    float dot = 0.0f, norm_a = 0.0f, norm_b = 0.0f;
    for (size_t i = 0; i < 32; ++i) {
        dot += a[i] * b[i];
        norm_a += a[i] * a[i];
        norm_b += b[i] * b[i];
    }
    float denom = std::sqrt(norm_a) * std::sqrt(norm_b) + EPS;
    return dot / denom;
}

Embedding PatternLibrary::computeCentroid(const std::vector<Embedding>& samples) {
    Embedding centroid{};
    if (samples.empty()) return centroid;
    for (const auto& s : samples) {
        for (size_t i = 0; i < 32; ++i) {
            centroid[i] += s[i];
        }
    }
    float inv_n = 1.0f / static_cast<float>(samples.size());
    for (size_t i = 0; i < 32; ++i) {
        centroid[i] *= inv_n;
    }
    return centroid;
}

float PatternLibrary::computeIntraVariance(const std::vector<Embedding>& samples, const Embedding& centroid) {
    if (samples.empty()) return 0.0f;
    float var = 0.0f;
    for (const auto& s : samples) {
        float d_sq = 0.0f;
        for (size_t i = 0; i < 32; ++i) {
            float d = s[i] - centroid[i];
            d_sq += d * d;
        }
        var += d_sq;
    }
    return var / static_cast<float>(samples.size());
}

PatternLibrary::PatternLibrary() {
    initializeDefaultPatterns();
}

void PatternLibrary::initializeDefaultPatterns() {
    std::lock_guard<std::mutex> lk(mutex_);
    patterns_.clear();

    struct PatternInit {
        CavitationType type;
        const char* name;
        const char* desc;
        Embedding (*centroid_fn)();
    };

    const PatternInit inits[] = {
        {CavitationType::CLOUD, "Cloud_Cavitation",
         "云空化: 低频宽带噪声+周期性冲击，云团状空泡群，伴随强烈冲击压力",
         createCloudCentroid},
        {CavitationType::SHEET, "Sheet_Cavitation",
         "片空化: 中高频稳定+准稳态，附着型片状空泡，边界层分离",
         createSheetCentroid},
        {CavitationType::SUPER, "Super_Cavitation",
         "超空化: 超高频窄带+极端强度，全面包裹空泡，减阻效应显著",
         createSuperCentroid},
        {CavitationType::VORTEX, "Vortex_Cavitation",
         "尾涡: 轴频率次谐波+调制边带，螺旋状涡空化，中心低压区",
         createVortexCentroid},
    };

    for (const auto& init : inits) {
        AcousticPattern p;
        p.timestamp = currentTimestampMs();
        p.cavitation_type = init.type;
        p.pattern_name = init.name;
        p.description = init.desc;

        Embedding centroid = init.centroid_fn();
        auto samples = generatePatternSamples(centroid, 10, 0.06f);

        p.embedding.assign(centroid.begin(), centroid.end());
        p.centroid.assign(centroid.begin(), centroid.end());

        p.samples.reserve(samples.size());
        for (const auto& s : samples) {
            p.samples.emplace_back(s.begin(), s.end());
        }

        p.sample_count = static_cast<uint32_t>(samples.size());
        p.intra_cluster_variance = computeIntraVariance(samples, centroid);
        p.silhouette_score = 0.85f;
        p.is_verified_by_expert = true;
        p.expert_note = "标准模式库预置样本";
        p.last_updated = currentTimestampMs();

        patterns_[init.type] = p;
    }
}

PatternMatchResult PatternLibrary::match(const Embedding& embedding) {
    auto top = matchTopN(embedding, 4);
    if (top.empty()) {
        return {CavitationType::UNKNOWN, "Unknown", 0.0f, 0.0f, false};
    }
    return top[0];
}

std::vector<PatternMatchResult> PatternLibrary::matchTopN(const Embedding& embedding, size_t n) {
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<PatternMatchResult> results;
    results.reserve(patterns_.size());

    for (const auto& [type, pattern] : patterns_) {
        Embedding p_emb{};
        size_t copy_n = std::min(pattern.centroid.size(), size_t(32));
        std::copy_n(pattern.centroid.begin(), copy_n, p_emb.begin());

        float sim = cosineSimilarity(embedding, p_emb);

        float dist = 0.0f;
        for (size_t i = 0; i < 32; ++i) {
            float d = embedding[i] - p_emb[i];
            dist += d * d;
        }
        dist = std::sqrt(dist);

        results.push_back({type, pattern.pattern_name, sim, dist, sim >= similarity_threshold_});
    }

    std::sort(results.begin(), results.end(),
              [](const auto& a, const auto& b) { return a.similarity > b.similarity; });

    if (results.size() > n) {
        results.resize(n);
    }

    return results;
}

bool PatternLibrary::detectNovelty(const Embedding& embedding, uint8_t cluster_label) {
    (void)cluster_label;
    std::lock_guard<std::mutex> lk(mutex_);

    float max_sim = 0.0f;
    for (const auto& [type, pattern] : patterns_) {
        Embedding p_emb{};
        size_t copy_n = std::min(pattern.centroid.size(), size_t(32));
        std::copy_n(pattern.centroid.begin(), copy_n, p_emb.begin());
        float sim = cosineSimilarity(embedding, p_emb);
        if (sim > max_sim) max_sim = sim;
    }

    return (1.0f - max_sim) > novelty_threshold_;
}

bool PatternLibrary::addPattern(
    CavitationType cavitation_type,
    const std::string& name,
    const std::vector<Embedding>& samples,
    bool is_expert_verified,
    const std::string& expert_note) {

    if (samples.empty()) return false;

    std::lock_guard<std::mutex> lk(mutex_);

    Embedding centroid = computeCentroid(samples);
    float variance = computeIntraVariance(samples, centroid);

    AcousticPattern p;
    p.timestamp = currentTimestampMs();
    p.cavitation_type = cavitation_type;
    p.pattern_name = name;
    p.description = cavitationTypeDescription(cavitation_type);
    p.embedding.assign(centroid.begin(), centroid.end());
    p.centroid.assign(centroid.begin(), centroid.end());

    p.samples.reserve(samples.size());
    for (const auto& s : samples) {
        p.samples.emplace_back(s.begin(), s.end());
    }

    p.sample_count = static_cast<uint32_t>(samples.size());
    p.intra_cluster_variance = variance;
    p.silhouette_score = 0.0f;
    p.is_verified_by_expert = is_expert_verified;
    p.expert_note = expert_note;
    p.last_updated = currentTimestampMs();

    patterns_[cavitation_type] = p;
    return true;
}

const AcousticPattern& PatternLibrary::getPattern(CavitationType type) const {
    std::lock_guard<std::mutex> lk(mutex_);
    static AcousticPattern empty{};
    auto it = patterns_.find(type);
    if (it != patterns_.end()) return it->second;
    return empty;
}

bool PatternLibrary::hasPattern(CavitationType type) const {
    std::lock_guard<std::mutex> lk(mutex_);
    return patterns_.count(type) > 0;
}

void PatternLibrary::updatePatternCentroid(CavitationType type, const Embedding& new_centroid) {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = patterns_.find(type);
    if (it != patterns_.end()) {
        it->second.centroid.assign(new_centroid.begin(), new_centroid.end());
        it->second.embedding.assign(new_centroid.begin(), new_centroid.end());
        it->second.last_updated = currentTimestampMs();
    }
}

std::string PatternLibrary::cavitationTypeDescription(CavitationType type) {
    switch (type) {
        case CavitationType::CLOUD:
            return "云空化: 低频宽带噪声+周期性冲击，云团状空泡群，伴随强烈冲击压力";
        case CavitationType::SHEET:
            return "片空化: 中高频稳定+准稳态，附着型片状空泡，边界层分离";
        case CavitationType::SUPER:
            return "超空化: 超高频窄带+极端强度，全面包裹空泡，减阻效应显著";
        case CavitationType::VORTEX:
            return "尾涡: 轴频率次谐波+调制边带，螺旋状涡空化，中心低压区";
        case CavitationType::TIP_LEAKAGE:
            return "叶顶泄漏: 叶尖间隙流动，泄漏涡空化，高频特征";
        default:
            return "未知空化类型";
    }
}

AcousticDiagnosisFacade::AcousticDiagnosisFacade()
    : triplet_miner_(0.3f),
      clustering_(10, 0.5f, 5) {}

bool AcousticDiagnosisFacade::init(const Config::DiagnosisConfig& config) {
    std::lock_guard<std::mutex> lk(mutex_);
    config_ = config;

    triplet_miner_.setMargin(config_.triplet_margin);
    clustering_ = OnlineKMeansClustering(
        config_.max_clusters,
        config_.cluster_merge_threshold,
        config_.min_samples_per_cluster);
    pattern_library_.setSimilarityThreshold(config_.similarity_threshold);
    pattern_library_.setNoveltyThreshold(config_.novelty_threshold);
    pattern_library_.initializeDefaultPatterns();

    network_.loadWeights(config_.pattern_library_path);
    if (!network_.isLoaded()) {
        network_.setDefaultWeights();
    }

    last_update_ts_ = currentTimestampMs();
    return true;
}

FeatureVector AcousticDiagnosisFacade::buildFeatureVector(const IPCMessageFeatures& msg) {
    FeatureVector fv{};
    size_t idx = 0;

    for (size_t i = 0; i < 11 && idx < 32; ++i) {
        fv[idx++] = msg.spectrum[i];
    }

    for (size_t i = 0; i < 16 && idx < 32; ++i) {
        fv[idx++] = msg.wavelet_energy[i];
    }

    if (idx < 32) fv[idx++] = msg.wavelet_entropy;
    if (idx < 32) fv[idx++] = msg.wavelet_total;
    if (idx < 32) fv[idx++] = msg.rms_value;

    float norm = 0.0f;
    for (size_t i = 0; i < 32; ++i) norm += fv[i] * fv[i];
    float inv_norm = 1.0f / (std::sqrt(norm) + EPS);
    for (size_t i = 0; i < 32; ++i) fv[i] *= inv_norm;

    return fv;
}

std::string AcousticDiagnosisFacade::cavitationTypeName(CavitationType type) {
    switch (type) {
        case CavitationType::CLOUD:    return "Cloud";
        case CavitationType::SHEET:    return "Sheet";
        case CavitationType::SUPER:    return "Super";
        case CavitationType::VORTEX:   return "Vortex";
        case CavitationType::TIP_LEAKAGE: return "TipLeakage";
        default:                      return "Unknown";
    }
}

Embedding AcousticDiagnosisFacade::extractEmbedding(const IPCMessageFeatures& feature_msg) {
    FeatureVector fv = buildFeatureVector(feature_msg);
    return network_.infer(fv);
}

PatternMatchResult AcousticDiagnosisFacade::matchKnown(const Embedding& embedding) {
    return pattern_library_.match(embedding);
}

uint8_t AcousticDiagnosisFacade::clusterUnknown(const Embedding& embedding) {
    uint8_t label = clustering_.assign(embedding);
    clustering_.update(embedding, label);
    return label;
}

std::vector<float> AcousticDiagnosisFacade::computeConfidenceScores(
    const Embedding& embedding,
    const PatternMatchResult& match_result,
    uint8_t cluster_label) {

    std::vector<float> scores(4, 0.0f);

    scores[0] = match_result.similarity;

    if (match_result.is_known) {
        float inv_dist = 1.0f / (match_result.centroid_distance + EPS);
        scores[1] = std::min(inv_dist * 0.5f, 1.0f);
    } else {
        float centroid_dist = clustering_.distanceToCentroid(embedding, cluster_label);
        scores[1] = std::max(0.0f, 1.0f - centroid_dist);
    }

    if (match_result.is_known) {
        scores[2] = match_result.similarity > config_.similarity_threshold ? 0.9f : match_result.similarity;
    } else {
        uint8_t cc = clustering_.clusterCount();
        scores[2] = (cc > 0) ? 0.5f + 0.3f * (1.0f - 1.0f / (1.0f + static_cast<float>(cc))) : 0.2f;
    }

    float w_sum = 0.5f * scores[0] + 0.3f * scores[1] + 0.2f * scores[2];
    scores[3] = std::clamp(w_sum, 0.0f, 1.0f);

    return scores;
}

DiagnosisResult AcousticDiagnosisFacade::diagnose(const IPCMessageFeatures& feature_msg) {
    uint64_t t0 = currentTimestampMs();

    std::lock_guard<std::mutex> lk(mutex_);

    DiagnosisResult result;
    result.timestamp = feature_msg.timestamp;
    result.turbine_id = feature_msg.turbine_id;
    result.sensor_id = feature_msg.sensor_id;
    result.cavitation_type = CavitationType::UNKNOWN;
    result.status = DiagnosisStatus::EXTRACTING;
    result.cluster_label = 0;
    result.is_known_pattern = false;
    result.embedding.resize(32, 0.0f);
    result.pattern_similarity.resize(4, 0.0f);
    result.confidence_scores.resize(4, 0.0f);
    result.centroid_distance = 0.0f;
    result.silhouette_score = 0.0f;
    result.cluster_purity = 0.0f;
    result.cavitation_type_name = "Unknown";
    result.expert_note = "";
    result.analysis_latency_us = 0;

    Embedding embedding = extractEmbedding(feature_msg);
    result.status = DiagnosisStatus::MATCHING;
    for (size_t i = 0; i < 32; ++i) {
        result.embedding[i] = embedding[i];
    }

    auto top_matches = pattern_library_.matchTopN(embedding, 4);
    for (size_t i = 0; i < top_matches.size() && i < 4; ++i) {
        result.pattern_similarity[i] = top_matches[i].similarity;
    }
    PatternMatchResult match = top_matches.empty()
        ? PatternMatchResult{CavitationType::UNKNOWN, "Unknown", 0.0f, 0.0f, false}
        : top_matches[0];

    total_diagnosed_++;

    if (match.is_known) {
        known_matches_++;
        result.cavitation_type = match.cavitation_type;
        result.cavitation_type_name = cavitationTypeName(match.cavitation_type);
        result.is_known_pattern = true;
        result.centroid_distance = match.centroid_distance;
        result.cluster_label = static_cast<uint8_t>(match.cavitation_type);
        result.expert_note = pattern_library_.cavitationTypeDescription(match.cavitation_type);
    } else {
        result.status = DiagnosisStatus::CLUSTERING;
        unknown_patterns_++;
        uint8_t cluster_label = clusterUnknown(embedding);
        result.cluster_label = cluster_label;
        result.cavitation_type = CavitationType::UNKNOWN;
        result.cavitation_type_name = "Novel_Cluster_" + std::to_string(cluster_label);
        result.is_known_pattern = false;
        result.centroid_distance = clustering_.distanceToCentroid(embedding, cluster_label);
        bool is_novel = pattern_library_.detectNovelty(embedding, cluster_label);
        result.expert_note = is_novel
            ? "检测到新声纹模式，建议专家审核确认"
            : "未匹配到已知模式，聚类分配中";
    }

    result.confidence_scores = computeConfidenceScores(embedding, match, result.cluster_label);
    result.status = DiagnosisStatus::COMPLETED;

    uint64_t t1 = currentTimestampMs();
    result.analysis_latency_us = (t1 - t0) * 1000;

    return result;
}

void AcousticDiagnosisFacade::periodicUpdate() {
    std::lock_guard<std::mutex> lk(mutex_);
    uint64_t now = currentTimestampMs();

    if (now - last_update_ts_ < config_.update_interval_s * 1000) {
        return;
    }

    clustering_.recomputeGlobalCentroids();
    clustering_.clusterMerge();

    for (const auto& cluster : clustering_.getClusters()) {
        if (cluster.sample_count >= config_.min_samples_per_cluster) {
            bool matches_known = false;
            for (const auto& [type, pattern] : pattern_library_.getPatterns()) {
                Embedding p_emb{};
                size_t copy_n = std::min(pattern.centroid.size(), size_t(32));
                std::copy_n(pattern.centroid.begin(), copy_n, p_emb.begin());

                float d = 0.0f;
                for (size_t i = 0; i < 32; ++i) {
                    float diff = cluster.centroid[i] - p_emb[i];
                    d += diff * diff;
                }
                d = std::sqrt(d);

                if (d < config_.cluster_merge_threshold) {
                    matches_known = true;
                    break;
                }
            }

            if (!matches_known && cluster.sample_count >= config_.min_samples_per_cluster * 2) {
                std::vector<Embedding> cluster_samples;
                cluster_samples.reserve(cluster.samples.size());
                for (const auto& s : cluster.samples) {
                    cluster_samples.push_back(s);
                }

                CavitationType new_type = static_cast<CavitationType>(
                    static_cast<uint8_t>(CavitationType::VORTEX) + 10 + cluster.label);
                std::string name = "AutoCluster_" + std::to_string(cluster.label);
                pattern_library_.addPattern(new_type, name, cluster_samples, false,
                    "自动聚类生成模式，待专家验证");
            }
        }
    }

    last_update_ts_ = now;
}

}
