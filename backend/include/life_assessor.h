#pragma once

#include <vector>
#include <deque>
#include <mutex>
#include "data_structures.h"

namespace turbine_monitor {

struct RainflowCycle {
    float range;
    float mean;
    uint32_t count;
};

struct MaterialProperties {
    std::string name;
    float ultimateTensileStrength;
    float fatigueLimit;
    float fractureToughness;
    float k;
    float m;
};

class LifeAssessor {
public:
    LifeAssessor();
    ~LifeAssessor() = default;

    BladeStress computeStress(
        const std::vector<float>& vibrationSignal,
        float cavitationIntensity,
        uint64_t timestamp,
        uint8_t turbineId,
        uint8_t bladeId,
        float sensitivity = 1.0f,
        float amplificationFactor = 50.0f);

    LifeAssessment assessLife(
        const BladeStress& stressData,
        const CavitationState& cavitationState,
        uint8_t turbineId,
        uint8_t bladeId,
        float cumulativeDamage = 0.0f);

    static std::vector<RainflowCycle> rainflowCounting(const std::vector<float>& stressTimeHistory);

    void setMaterialProperties(const MaterialProperties& props);
    const MaterialProperties& getMaterialProperties() const;

    void setExpectedLifeHours(float hours);
    float getExpectedLifeHours() const;

    void resetCumulativeDamage(uint8_t turbineId, uint8_t bladeId);

private:
    MaterialProperties materialProps_;
    float expectedLifeHours_;

    std::deque<std::vector<float>> stressHistory_;
    std::vector<std::vector<float>> cumulativeDamage_;
    mutable std::mutex mutex_;

    static std::vector<float> goodmanCorrection(const RainflowCycle& cycle, float ultimateStrength);
    static float computeCycleDamage(float stressRange, float meanStress,
                                     float k, float m, float ultimateStrength);
    static float minerSum(const std::vector<RainflowCycle>& cycles,
                          float k, float m, float ultimateStrength);

    float estimateRemainingLife(float cumulativeDamage, float currentDamageRate);
    float estimateDamageRate(const std::vector<RainflowCycle>& cycles,
                             float k, float m, float ultimateStrength);

    std::vector<float> buildStressTimeHistory(const std::vector<float>& vibrationSignal,
                                               float cavitationIntensity,
                                               float sensitivity,
                                               float amplificationFactor);

    static std::vector<float> computeStressAmplitude(const std::vector<float>& signal,
                                                      float baseStress,
                                                      float sensitivity,
                                                      float amplificationFactor);
};

}
