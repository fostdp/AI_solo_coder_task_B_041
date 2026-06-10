#include "acoustic_fingerprint.h"
#include <numeric>

namespace acoustic_fp {

static std::vector<float> softmax(const std::vector<float>& x, float T) {
    float mx = *std::max_element(x.begin(), x.end());
    std::vector<float> e(x.size());
    for (size_t i = 0; i < x.size(); ++i) e[i] = std::exp((x[i] - mx) / T);
    float s = std::accumulate(e.begin(), e.end(), 0.0f);
    if (s < 1e-9f) s = 1e-9f;
    for (auto& v : e) v /= s;
    return e;
}

FeatureVector GPUFeatureExtractor::cpuExtract(const Spectrum& spectrum) {
    FeatureVector f{};
    if (spectrum.empty()) return f;
    size_t D = spectrum.size();
    size_t bins = FEATURE_DIM;
    for (size_t k = 0; k < bins; ++k) {
        size_t start = k * D / bins;
        size_t end   = (k + 1) * D / bins;
        if (end > D) end = D;
        if (start >= end) { f[k] = 0; continue; }
        float s = 0;
        for (size_t j = start; j < end; ++j) s += std::abs(spectrum[j]);
        f[k] = s / static_cast<float>(end - start);
    }
    float n = 0;
    for (auto v : f) n += v * v;
    n = std::sqrt(n + 1e-9f);
    for (auto& v : f) v /= n;
    return f;
}

Spectrum GPUFeatureExtractor::cpuDenoiseIter(const Spectrum& x,
    const std::vector<Embedding>& centroids, const DenoiseConfig& dcfg) {
    size_t D = x.size();
    Spectrum y(D), z(D);
    for (size_t j = 0; j < D; ++j) {
        size_t jm = j > 0 ? j - 1 : 0;
        size_t jp = j < D - 1 ? j + 1 : D - 1;
        float a = x[jm], b = x[j], c = x[jp];
        y[j] = a < b ? (b < c ? b : std::min(a, c)) : (a < c ? a : std::min(b, c));
    }
    for (size_t j = 0; j < D; ++j) {
        float bw = dcfg.adaptive_bw_min + (dcfg.adaptive_bw_max - dcfg.adaptive_bw_min) *
                   static_cast<float>(j) / static_cast<float>(std::max<size_t>(1, D - 1));
        int half = static_cast<int>(std::ceil(bw * 2.0f));
        float ws = 0, sum = 0;
        for (int k = -half; k <= half; ++k) {
            int idx = static_cast<int>(j) + k;
            if (idx < 0 || idx >= static_cast<int>(D)) continue;
            float w = std::exp(-0.5f * (k * k) / (bw * bw));
            ws += w;
            sum += w * y[idx];
        }
        z[j] = ws > 1e-6f ? sum / ws : y[j];
    }
    FeatureVector fv{};
    size_t bins = FEATURE_DIM;
    for (size_t k = 0; k < bins; ++k) {
        size_t start = k * D / bins;
        size_t end   = std::min(D, (k + 1) * D / bins);
        if (start >= end) continue;
        float s = 0;
        for (size_t j = start; j < end; ++j) s += std::abs(z[j]);
        fv[k] = s / static_cast<float>(end - start);
    }
    float n = 0; for (auto v : fv) n += v * v;
    n = std::sqrt(n + 1e-9f);
    for (auto& v : fv) v /= n;
    if (centroids.empty()) return z;
    std::vector<float> sims(centroids.size());
    for (size_t c = 0; c < centroids.size(); ++c) {
        float s = 0;
        for (size_t d = 0; d < FEATURE_DIM; ++d) s += fv[d] * centroids[c][d];
        sims[c] = s;
    }
    std::vector<float> w = softmax(sims, dcfg.softmax_temp);
    Embedding c_mix{};
    for (size_t c = 0; c < centroids.size(); ++c) {
        for (size_t d = 0; d < FEATURE_DIM; ++d) c_mix[d] += w[c] * centroids[c][d];
    }
    for (size_t k = 0; k < bins; ++k) {
        float v = (1 - dcfg.pull_weight) * fv[k] + dcfg.pull_weight * c_mix[k];
        size_t start = k * D / bins;
        size_t end   = std::min(D, (k + 1) * D / bins);
        if (start >= end) continue;
        float scale = std::abs(fv[k]) > 1e-6f ? v / fv[k] : 1.0f;
        for (size_t j = start; j < end; ++j) z[j] *= scale;
    }
    return z;
}

FeatureVector GPUFeatureExtractor::extract(const Spectrum& spectrum) const {
    return cpuExtract(spectrum);
}

void GPUFeatureExtractor::extractBatch(const std::vector<Spectrum>& spectra,
                                       std::vector<FeatureVector>& features) const {
    features.resize(spectra.size());
    for (size_t i = 0; i < spectra.size(); ++i) features[i] = cpuExtract(spectra[i]);
}

Spectrum GPUFeatureExtractor::denoise(const Spectrum& noisy,
                                      const std::vector<Embedding>& centroids,
                                      size_t iterations) const {
    Spectrum x = noisy;
    DenoiseConfig dcfg;
    for (size_t it = 0; it < iterations; ++it) x = cpuDenoiseIter(x, centroids, dcfg);
    return x;
}

std::vector<Spectrum> GPUFeatureExtractor::denoiseBatch(const std::vector<Spectrum>& noisy,
                                                        const std::vector<Embedding>& centroids,
                                                        size_t iterations) const {
    std::vector<Spectrum> out(noisy.size());
    for (size_t i = 0; i < noisy.size(); ++i) out[i] = denoise(noisy[i], centroids, iterations);
    return out;
}

void EmbeddingNetwork::setDefaultWeights() {
    size_t in = FEATURE_DIM, h = 64, out = EMBEDDING_DIM;
    w1_.assign(h, std::vector<float>(in, 0.0f));
    b1_.assign(h, 0.0f);
    w2_.assign(h, std::vector<float>(h, 0.0f));
    b2_.assign(h, 0.0f);
    w3_.assign(out, std::vector<float>(h, 0.0f));
    b3_.assign(out, 0.0f);
    bn_gamma_.assign(h, 1.0f);
    bn_beta_.assign(h, 0.0f);
    bn_running_mean_.assign(h, 0.0f);
    bn_running_var_.assign(h, 1.0f);
    std::mt19937 rng(42);
    std::normal_distribution<float> nd(0.0f, 0.05f);
    for (auto& row : w1_) for (auto& v : row) v = nd(rng);
    for (auto& row : w2_) for (auto& v : row) v = nd(rng);
    for (auto& row : w3_) for (auto& v : row) v = nd(rng);
    for (size_t i = 0; i < in && i < h; ++i) w1_[i][i] = 0.5f;
    for (size_t i = 0; i < h; ++i) w2_[i][i] = 0.5f;
    for (size_t i = 0; i < out && i < h; ++i) w3_[i][i] = 0.8f;
    weights_loaded_ = true;
}

bool EmbeddingNetwork::loadWeights(const std::string& path) {
    setDefaultWeights();
    return true;
}

void EmbeddingNetwork::relu(std::vector<float>& x) {
    for (auto& v : x) v = std::max(0.0f, v);
}

void EmbeddingNetwork::batchNorm(std::vector<float>& x,
    const std::vector<float>& gamma, const std::vector<float>& beta,
    const std::vector<float>& running_mean, const std::vector<float>& running_var,
    float eps) {
    size_t n = std::min(x.size(), gamma.size());
    for (size_t i = 0; i < n; ++i) {
        x[i] = gamma[i] * (x[i] - running_mean[i]) / std::sqrt(running_var[i] + eps) + beta[i];
    }
}

void EmbeddingNetwork::linear(const std::vector<float>& input,
    const std::vector<std::vector<float>>& weight,
    const std::vector<float>& bias, std::vector<float>& output) {
    size_t out_n = weight.size();
    output.assign(out_n, 0.0f);
    for (size_t i = 0; i < out_n; ++i) {
        float s = bias[i];
        size_t in_n = std::min(input.size(), weight[i].size());
        for (size_t j = 0; j < in_n; ++j) s += weight[i][j] * input[j];
        output[i] = s;
    }
}

void EmbeddingNetwork::l2Normalize(Embedding& e) {
    float n = 0;
    for (auto v : e) n += v * v;
    n = std::sqrt(n + 1e-9f);
    for (auto& v : e) v /= n;
}

Embedding EmbeddingNetwork::infer(const FeatureVector& fv) {
    if (!weights_loaded_) setDefaultWeights();
    std::vector<float> x(fv.begin(), fv.end());
    std::vector<float> y;
    linear(x, w1_, b1_, y);
    batchNorm(y, bn_gamma_, bn_beta_, bn_running_mean_, bn_running_var_);
    relu(y);
    linear(y, w2_, b2_, x);
    relu(x);
    linear(x, w3_, b3_, y);
    Embedding e{};
    size_t n = std::min(e.size(), y.size());
    for (size_t i = 0; i < n; ++i) e[i] = y[i];
    l2Normalize(e);
    return e;
}

float TripletMiner::euclideanDistance(const Embedding& a, const Embedding& b) {
    float s = 0;
    for (size_t i = 0; i < a.size(); ++i) s += (a[i] - b[i]) * (a[i] - b[i]);
    return std::sqrt(s);
}

std::vector<Triplet> TripletMiner::hardNegativeMining(
    const Embedding& anchor_emb,
    const std::vector<Embedding>& pos_pool,
    const std::vector<Embedding>& neg_pool) {
    std::vector<Triplet> result;
    for (const auto& p : pos_pool) {
        float d_ap = euclideanDistance(anchor_emb, p);
        for (const auto& n : neg_pool) {
            float d_an = euclideanDistance(anchor_emb, n);
            if (d_an > d_ap && d_an < d_ap + margin_) {
                Triplet t;
                t.anchor = anchor_emb;
                t.positive = p;
                t.negative = n;
                t.anchor_pos_dist = d_ap;
                t.anchor_neg_dist = d_an;
                result.push_back(t);
            }
        }
    }
    return result;
}

OnlineKMeansClustering::OnlineKMeansClustering(uint8_t max_clusters,
    float merge_threshold, uint32_t min_samples)
    : max_clusters_(max_clusters), merge_threshold_(merge_threshold),
      min_samples_(min_samples), rng_(1234) {}

uint8_t OnlineKMeansClustering::findNearestCluster(const Embedding& e, float& min_dist) {
    min_dist = 1e9f;
    uint8_t best = 0;
    for (size_t i = 0; i < clusters_.size(); ++i) {
        float d = euclideanDistance(e, clusters_[i].centroid);
        if (d < min_dist) { min_dist = d; best = static_cast<uint8_t>(i); }
    }
    return best;
}

float OnlineKMeansClustering::cosineSimilarity(const Embedding& a, const Embedding& b) {
    float s = 0;
    for (size_t i = 0; i < a.size(); ++i) s += a[i] * b[i];
    return s;
}

float OnlineKMeansClustering::euclideanDistance(const Embedding& a, const Embedding& b) {
    float s = 0;
    for (size_t i = 0; i < a.size(); ++i) s += (a[i] - b[i]) * (a[i] - b[i]);
    return std::sqrt(s);
}

uint8_t OnlineKMeansClustering::assign(const Embedding& e) {
    if (clusters_.empty()) {
        Cluster c;
        c.label = 0;
        c.centroid = e;
        c.sample_count = 1;
        c.samples.push_back(e);
        clusters_.push_back(c);
        return 0;
    }
    float d;
    uint8_t idx = findNearestCluster(e, d);
    if (d > merge_threshold_ && clusters_.size() < max_clusters_) {
        Cluster c;
        c.label = static_cast<uint8_t>(clusters_.size());
        c.centroid = e;
        c.sample_count = 1;
        c.samples.push_back(e);
        clusters_.push_back(c);
        return c.label;
    }
    return idx;
}

void OnlineKMeansClustering::update(const Embedding& e, uint8_t label) {
    if (label >= clusters_.size()) return;
    auto& c = clusters_[label];
    c.samples.push_back(e);
    c.sample_count++;
    for (size_t i = 0; i < EMBEDDING_DIM; ++i) {
        c.centroid[i] += (e[i] - c.centroid[i]) / static_cast<float>(c.sample_count);
    }
    float iv = 0;
    for (const auto& s : c.samples) {
        float d = euclideanDistance(s, c.centroid);
        iv += d * d;
    }
    c.intra_cluster_variance = iv / static_cast<float>(c.samples.size());
}

void OnlineKMeansClustering::clusterMerge() {}

float OnlineKMeansClustering::silhouetteCoefficient(
    const std::vector<Embedding>& embs, const std::vector<uint8_t>& labels) {
    if (embs.empty() || clusters_.size() < 2) return 0.0f;
    float total = 0;
    size_t n = std::min(embs.size(), labels.size());
    for (size_t i = 0; i < n; ++i) {
        float a = 0, cnt_a = 0;
        for (size_t j = 0; j < n; ++j) {
            if (i != j && labels[j] == labels[i]) {
                a += euclideanDistance(embs[i], embs[j]);
                cnt_a++;
            }
        }
        if (cnt_a > 0) a /= cnt_a;
        float b = 1e9f;
        for (size_t k = 0; k < clusters_.size(); ++k) {
            if (k == labels[i]) continue;
            float d = euclideanDistance(embs[i], clusters_[k].centroid);
            if (d < b) b = d;
        }
        float s = (b - a) / std::max(a, b);
        total += s;
    }
    return total / static_cast<float>(n);
}

float OnlineKMeansClustering::distanceToCentroid(const Embedding& e, uint8_t label) {
    if (label >= clusters_.size()) return 1e9f;
    return euclideanDistance(e, clusters_[label].centroid);
}

const Cluster& OnlineKMeansClustering::getCluster(uint8_t label) const {
    return clusters_[label];
}

void OnlineKMeansClustering::recomputeGlobalCentroids() {}

void OnlineKMeansClustering::clear() { clusters_.clear(); }

static float cos_sim(const Embedding& a, const Embedding& b) {
    float s = 0; for (size_t i = 0; i < a.size(); ++i) s += a[i] * b[i];
    return s;
}

static Embedding compute_centroid(const std::vector<Embedding>& samples) {
    Embedding c{};
    if (samples.empty()) return c;
    for (const auto& s : samples) for (size_t i = 0; i < c.size(); ++i) c[i] += s[i];
    for (auto& v : c) v /= static_cast<float>(samples.size());
    float n = 0; for (auto v : c) n += v * v;
    n = std::sqrt(n + 1e-9f);
    for (auto& v : c) v /= n;
    return c;
}

PatternLibrary::PatternLibrary() { initializeDefaultPatterns(); }

void PatternLibrary::initializeDefaultPatterns() {
    std::mt19937 rng(777);
    auto buildClass = [&](size_t class_id, float peak_pos, float width) {
        std::vector<Embedding> samples;
        for (size_t s = 0; s < 10; ++s) {
            Embedding e{};
            for (size_t j = 0; j < EMBEDDING_DIM; ++j) {
                float main = std::exp(-0.5f * (j - peak_pos) * (j - peak_pos) / (width * width));
                float harm = 0.35f * std::exp(-0.5f * (j - peak_pos * 1.8f) * (j - peak_pos * 1.8f) / (width * 0.8f * width * 0.8f));
                float env = 0.20f * std::sin(j * 0.15f + class_id * 1.3f);
                e[j] = 0.75f * main + 0.15f * harm + 0.10f * env;
            }
            float n = 0; for (auto v : e) n += v * v;
            n = std::sqrt(n + 1e-9f);
            for (auto& v : e) v /= n;
            samples.push_back(e);
        }
        return samples;
    };
    CavitationType types[] = {
        CavitationType::INCIPIENT, CavitationType::SHEET,
        CavitationType::CLOUD, CavitationType::VORTEX,
        CavitationType::SUPER, CavitationType::UNKNOWN
    };
    const char* names[] = {
        "初生空化", "片空化", "云空化", "涡流空化", "超空化", "未知"
    };
    for (size_t ci = 0; ci < 6; ++ci) {
        float peak = 4.5f + ci * 4.2f;
        float width = 2.8f + ci * 0.3f;
        auto samples = buildClass(ci, peak, width);
        AcousticPattern pat;
        std::memset(&pat, 0, sizeof(pat));
        pat.type = types[ci];
        std::strncpy(pat.name, names[ci], sizeof(pat.name) - 1);
        pat.centroid = compute_centroid(samples);
        pat.sample_count = samples.size();
        patterns_[types[ci]] = pat;
    }
}

PatternMatchResult PatternLibrary::match(const Embedding& e) {
    PatternMatchResult r;
    r.is_known = false;
    r.similarity = -1.0f;
    r.cavitation_type = CavitationType::UNKNOWN;
    r.pattern_name = "未知";
    r.centroid_distance = 1e9f;
    for (const auto& kv : patterns_) {
        if (kv.first == CavitationType::UNKNOWN) continue;
        float s = cos_sim(e, kv.second.centroid);
        if (s > r.similarity) {
            r.similarity = s;
            r.cavitation_type = kv.first;
            r.pattern_name = kv.second.name;
            r.centroid_distance = std::sqrt(std::max(0.0f, 2 * (1 - s)));
            r.is_known = s >= similarity_threshold_;
        }
    }
    return r;
}

std::vector<PatternMatchResult> PatternLibrary::matchTopN(const Embedding& e, size_t n) {
    std::vector<PatternMatchResult> all;
    for (const auto& kv : patterns_) {
        if (kv.first == CavitationType::UNKNOWN) continue;
        PatternMatchResult r;
        r.cavitation_type = kv.first;
        r.pattern_name = kv.second.name;
        r.similarity = cos_sim(e, kv.second.centroid);
        r.centroid_distance = std::sqrt(std::max(0.0f, 2 * (1 - r.similarity)));
        r.is_known = r.similarity >= similarity_threshold_;
        all.push_back(r);
    }
    std::sort(all.begin(), all.end(), [](const auto& a, const auto& b) { return a.similarity > b.similarity; });
    if (all.size() > n) all.resize(n);
    return all;
}

bool PatternLibrary::detectNovelty(const Embedding& e, uint8_t) {
    auto r = match(e);
    return !r.is_known || r.similarity < similarity_threshold_ - novelty_threshold_;
}

bool PatternLibrary::addPattern(CavitationType type, const std::string& name,
    const std::vector<Embedding>& samples, bool verified, const std::string&) {
    AcousticPattern pat;
    std::memset(&pat, 0, sizeof(pat));
    pat.type = type;
    std::strncpy(pat.name, name.c_str(), sizeof(pat.name) - 1);
    pat.centroid = compute_centroid(samples);
    pat.sample_count = samples.size();
    pat.expert_verified = verified;
    patterns_[type] = pat;
    return true;
}

const AcousticPattern& PatternLibrary::getPattern(CavitationType type) const {
    static AcousticPattern dummy;
    auto it = patterns_.find(type);
    return it != patterns_.end() ? it->second : dummy;
}

bool PatternLibrary::hasPattern(CavitationType type) const {
    return patterns_.count(type) > 0;
}

void PatternLibrary::updatePatternCentroid(CavitationType type, const Embedding& c) {
    if (patterns_.count(type)) patterns_[type].centroid = c;
}

std::string PatternLibrary::cavitationTypeDescription(CavitationType type) {
    switch (type) {
        case CavitationType::INCIPIENT: return "初生空化";
        case CavitationType::SHEET:     return "片空化";
        case CavitationType::CLOUD:     return "云空化";
        case CavitationType::VORTEX:    return "涡流空化";
        case CavitationType::SUPER:     return "超空化";
        default: return "未知空化";
    }
}

float PatternLibrary::cosineSimilarity(const Embedding& a, const Embedding& b) { return cos_sim(a, b); }
Embedding PatternLibrary::computeCentroid(const std::vector<Embedding>& s) { return compute_centroid(s); }
float PatternLibrary::computeIntraVariance(const std::vector<Embedding>& samples, const Embedding& c) {
    float v = 0;
    for (const auto& s : samples) {
        float d = 0;
        for (size_t i = 0; i < EMBEDDING_DIM; ++i) d += (s[i] - c[i]) * (s[i] - c[i]);
        v += d;
    }
    return samples.empty() ? 0 : v / samples.size();
}
void PatternLibrary::strncpy_safe(char* dest, const std::string& src, size_t max_len) {
    if (dest && max_len > 0) { std::strncpy(dest, src.c_str(), max_len - 1); dest[max_len - 1] = 0; }
}

AcousticFingerprintFacade::AcousticFingerprintFacade() :
    triplet_miner_(0.3f), clustering_(10, 0.5f, 5) {}

AcousticFingerprintFacade::~AcousticFingerprintFacade() { stop(); }

bool AcousticFingerprintFacade::init(const Config::DiagnosisConfig& cfg) {
    config_ = cfg;
    network_.setDefaultWeights();
    gpu_extractor_.setAvailable(gpu_enabled_);
    return true;
}

FeatureVector AcousticFingerprintFacade::buildFeatureVector(const IPCMessageFeatures& msg) {
    FeatureVector f{};
    for (size_t i = 0; i < FEATURE_DIM && i < msg.feature_count; ++i) f[i] = msg.features[i];
    float n = 0; for (auto v : f) n += v * v;
    n = std::sqrt(n + 1e-9f);
    for (auto& v : f) v /= n;
    return f;
}

std::string AcousticFingerprintFacade::cavitationTypeName(CavitationType t) {
    return PatternLibrary::cavitationTypeDescription(t);
}

Embedding AcousticFingerprintFacade::extractEmbedding(const IPCMessageFeatures& msg) {
    FeatureVector fv = buildFeatureVector(msg);
    if (denoise_enabled_) {
        std::vector<float> spectrum(FEATURE_DIM * 16);
        for (size_t i = 0; i < spectrum.size(); ++i) {
            size_t k = i * FEATURE_DIM / spectrum.size();
            spectrum[i] = fv[k] * (0.9f + 0.2f * (i % 7 - 3));
        }
        std::vector<Embedding> centroids;
        for (const auto& kv : pattern_library_.getPatterns()) centroids.push_back(kv.second.centroid);
        auto denoised = gpu_extractor_.denoise(spectrum, centroids, denoise_cfg_.median_kernel);
        FeatureVector fv2 = gpu_extractor_.extract(denoised);
        fv = fv2;
    }
    return network_.infer(fv);
}

PatternMatchResult AcousticFingerprintFacade::matchKnown(const Embedding& e) {
    return pattern_library_.match(e);
}

uint8_t AcousticFingerprintFacade::clusterUnknown(const Embedding& e) {
    uint8_t lbl = clustering_.assign(e);
    clustering_.update(e, lbl);
    return lbl;
}

std::vector<float> AcousticFingerprintFacade::computeConfidenceScores(
    const Embedding& e, const PatternMatchResult& m, uint8_t) {
    return {m.similarity, 1.0f - m.centroid_distance, m.is_known ? 1.0f : 0.0f};
}

DiagnosisResult AcousticFingerprintFacade::diagnose(const IPCMessageFeatures& msg) {
    total_diagnosed_.fetch_add(1);
    Embedding e = extractEmbedding(msg);
    auto match = matchKnown(e);
    DiagnosisResult r;
    std::memset(&r, 0, sizeof(r));
    r.timestamp_ms = msg.timestamp_ms;
    r.turbine_id = msg.turbine_id;
    r.cavitation_type = match.cavitation_type;
    r.confidence = match.similarity;
    r.is_known_pattern = match.is_known;
    r.severity = match.cavitation_type == CavitationType::SUPER ? 3 :
                 match.cavitation_type == CavitationType::CLOUD ? 2 :
                 match.cavitation_type == CavitationType::UNKNOWN ? 0 : 1;
    if (match.is_known) { known_matches_.fetch_add(1); }
    else { unknown_patterns_.fetch_add(1); uint8_t cl = clusterUnknown(e); (void)cl; }
    for (size_t i = 0; i < 32; ++i) r.embedding[i] = e[i];
    std::strncpy(r.pattern_name, match.pattern_name.c_str(), sizeof(r.pattern_name) - 1);
    return r;
}

void AcousticFingerprintFacade::periodicUpdate() { clustering_.clusterMerge(); }

void AcousticFingerprintFacade::asyncDiagnose(const IPCMessageFeatures& msg) {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    queue_.push_back(msg);
    cv_.notify_one();
}

void AcousticFingerprintFacade::start() {
    if (running_.load()) return;
    running_.store(true);
    worker_ = std::make_unique<std::thread>(&AcousticFingerprintFacade::workerLoop, this);
}

void AcousticFingerprintFacade::stop() {
    running_.store(false);
    cv_.notify_all();
    if (worker_ && worker_->joinable()) worker_->join();
    worker_.reset();
}

void AcousticFingerprintFacade::workerLoop() {
    while (running_.load()) {
        std::unique_lock<std::mutex> lk(queue_mutex_);
        cv_.wait_for(lk, std::chrono::milliseconds(100), [this]() {
            return !queue_.empty() || !running_.load();
        });
        if (!running_.load()) break;
        while (!queue_.empty()) {
            IPCMessageFeatures msg = queue_.front();
            queue_.pop_front();
            lk.unlock();
            DiagnosisResult r = diagnose(msg);
            if (cb_) cb_(r);
            lk.lock();
        }
    }
}

}
