#pragma once

#include <vector>
#include <string>
#include <memory>
#include <random>
#include <mutex>
#include "data_structures.h"

namespace turbine_monitor {

class IsolationForest {
public:
    struct Node {
        bool isLeaf;
        int featureIndex;
        double threshold;
        int leftChild;
        int rightChild;
        double depth;
    };

    struct Tree {
        std::vector<Node> nodes;
        int root;
    };

    IsolationForest(int numTrees = 100, int subSamplingSize = 256);
    ~IsolationForest() = default;

    void fit(const std::vector<std::vector<float>>& data);
    double anomalyScore(const std::vector<float>& sample) const;
    bool loadModel(const std::string& path);
    bool saveModel(const std::string& path) const;

private:
    int numTrees_;
    int subSamplingSize_;
    std::vector<Tree> trees_;
    mutable std::mt19937 rng_;
    mutable std::mutex mutex_;

    int buildTree(const std::vector<std::vector<float>>& data,
                  const std::vector<int>& indices,
                  int depth,
                  Tree& tree);

    double pathLength(const std::vector<float>& sample,
                      const Tree& tree,
                      int nodeIndex) const;

    static double harmonicNumber(int n);
    static double cFactor(int n);
};

class AutoEncoder {
public:
    struct Layer {
        std::vector<std::vector<float>> weights;
        std::vector<float> biases;
        std::string activation;
    };

    AutoEncoder();
    ~AutoEncoder() = default;

    bool loadModel(const std::string& path);
    std::vector<float> forward(const std::vector<float>& input) const;
    double reconstructionError(const std::vector<float>& input) const;

private:
    std::vector<Layer> encoder_;
    std::vector<Layer> decoder_;
    int inputSize_;
    int latentSize_;
    mutable std::mutex mutex_;

    static std::vector<float> activate(const std::vector<float>& x, const std::string& activation);
    static std::vector<float> fullyConnected(const std::vector<float>& input,
                                             const Layer& layer);
};

class CavitationDetector {
public:
    CavitationDetector(bool enableAutoEncoder = true,
                       bool enableIsolationForest = true,
                       const std::string& autoencoderPath = "",
                       const std::string& isolationForestPath = "");
    ~CavitationDetector() = default;

    CavitationState detect(
        const SpectrumFeatures& spectrum,
        const WaveletFeatures& wavelet,
        uint64_t timestamp,
        uint8_t turbineId,
        uint8_t bladeId);

    bool loadModels();
    void setThresholds(float incipientThreshold, float criticalThreshold, float developedThreshold);

private:
    std::unique_ptr<AutoEncoder> autoEncoder_;
    std::unique_ptr<IsolationForest> isolationForest_;
    bool enableAutoEncoder_;
    bool enableIsolationForest_;
    std::string autoencoderPath_;
    std::string isolationForestPath_;

    float incipientThreshold_;
    float criticalThreshold_;
    float developedThreshold_;

    std::vector<float> buildFeatureVector(
        const SpectrumFeatures& spectrum,
        const WaveletFeatures& wavelet);

    static void normalizeFeatures(std::vector<float>& features);

    CavitationStage classifyStage(float combinedScore);
    float computeCavitationIntensity(float combinedScore);
    float computeConfidence(float combinedScore, ModelType modelType);
};

}
