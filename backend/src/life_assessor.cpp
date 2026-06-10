#include "life_assessor.h"
#include <algorithm>
#include <cmath>
#include <deque>
#include <numeric>

namespace turbine_monitor {

LifeAssessor::LifeAssessor()
    : expectedLifeHours_(200000.0f) {
    materialProps_ = {
        "13Cr4Ni",
        750.0f,
        250.0f,
        60.0f,
        5.0e-12f,
        3.0f
    };

    cumulativeDamage_.resize(TURBINE_COUNT + 1);
    for (auto& blades : cumulativeDamage_) {
        blades.resize(BLADE_COUNT + 1, 0.0f);
    }
}

void LifeAssessor::setMaterialProperties(const MaterialProperties& props) {
    std::lock_guard<std::mutex> lock(mutex_);
    materialProps_ = props;
}

const MaterialProperties& LifeAssessor::getMaterialProperties() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return materialProps_;
}

void LifeAssessor::setExpectedLifeHours(float hours) {
    std::lock_guard<std::mutex> lock(mutex_);
    expectedLifeHours_ = hours;
}

float LifeAssessor::getExpectedLifeHours() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return expectedLifeHours_;
}

void LifeAssessor::resetCumulativeDamage(uint8_t turbineId, uint8_t bladeId) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (turbineId > 0 && turbineId <= TURBINE_COUNT &&
        bladeId > 0 && bladeId <= BLADE_COUNT) {
        cumulativeDamage_[turbineId][bladeId] = 0.0f;
    }
}

std::vector<float> LifeAssessor::computeStressAmplitude(
    const std::vector<float>& signal,
    float baseStress,
    float sensitivity,
    float amplificationFactor) {

    std::vector<float> stress(signal.size());

    float rms = 0.0f;
    for (float v : signal) {
        rms += v * v;
    }
    rms = std::sqrt(rms / signal.size());

    float scaleFactor = rms > 0.0f ? (sensitivity * amplificationFactor) / rms : 1.0f;

    for (size_t i = 0; i < signal.size(); ++i) {
        stress[i] = baseStress + signal[i] * scaleFactor;
    }

    return stress;
}

std::vector<float> LifeAssessor::buildStressTimeHistory(
    const std::vector<float>& vibrationSignal,
    float cavitationIntensity,
    float sensitivity,
    float amplificationFactor) {

    float baseStress = 50.0f + cavitationIntensity * 100.0f;
    return computeStressAmplitude(vibrationSignal, baseStress, sensitivity, amplificationFactor);
}

std::vector<RainflowCycle> LifeAssessor::rainflowCounting(
    const std::vector<float>& stressTimeHistory) {

    std::vector<RainflowCycle> cycles;
    if (stressTimeHistory.size() < 3) return cycles;

    std::deque<float> points;
    for (float s : stressTimeHistory) {
        points.push_back(s);
    }

    std::deque<float> stack;
    size_t i = 0;

    while (i < points.size()) {
        stack.push_back(points[i]);

        while (stack.size() >= 4) {
            size_t n = stack.size();
            float s0 = stack[n - 4];
            float s1 = stack[n - 3];
            float s2 = stack[n - 2];
            float s3 = stack[n - 1];

            float range1 = std::abs(s1 - s2);
            float range2 = std::abs(s0 - s1);
            float range3 = std::abs(s2 - s3);

            if (range1 <= range2 && range1 <= range3) {
                float mean = (s1 + s2) / 2.0f;
                float range = std::abs(s2 - s1);

                bool found = false;
                for (auto& cycle : cycles) {
                    if (std::abs(cycle.range - range) < 1e-6 &&
                        std::abs(cycle.mean - mean) < 1e-6) {
                        cycle.count++;
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    cycles.push_back({range, mean, 1});
                }

                float s_removed = s1;
                stack.erase(stack.end() - 3, stack.end() - 1);

                if (!stack.empty() && std::abs(stack.back() - s_removed) < 1e-6) {
                    stack.pop_back();
                }
            } else {
                break;
            }
        }
        i++;
    }

    while (stack.size() >= 2) {
        float s1 = stack[0];
        float s2 = stack[1];
        float mean = (s1 + s2) / 2.0f;
        float range = std::abs(s2 - s1);

        if (range > 0) {
            bool found = false;
            for (auto& cycle : cycles) {
                if (std::abs(cycle.range - range) < 1e-6 &&
                    std::abs(cycle.mean - mean) < 1e-6) {
                    cycle.count++;
                    found = true;
                    break;
                }
            }
            if (!found) {
                cycles.push_back({range, mean, 1});
            }
        }
        stack.pop_front();
    }

    return cycles;
}

std::vector<float> LifeAssessor::goodmanCorrection(
    const RainflowCycle& cycle,
    float ultimateStrength) {

    float stressAmplitude = cycle.range / 2.0f;
    float meanStress = cycle.mean;

    float correctedAmplitude = stressAmplitude;
    if (meanStress > 0 && ultimateStrength > meanStress) {
        correctedAmplitude = stressAmplitude * ultimateStrength / (ultimateStrength - meanStress);
    }

    return {correctedAmplitude, meanStress};
}

float LifeAssessor::computeCycleDamage(
    float stressRange,
    float meanStress,
    float k,
    float m,
    float ultimateStrength) {

    float correctedRange = stressRange;
    if (meanStress > 0 && ultimateStrength > meanStress) {
        correctedRange = stressRange * ultimateStrength / (ultimateStrength - meanStress);
    }

    if (correctedRange <= 0) return 0.0f;

    float cyclesToFailure = k * std::pow(correctedRange, -m);
    return cyclesToFailure > 0 ? 1.0f / cyclesToFailure : 0.0f;
}

float LifeAssessor::minerSum(
    const std::vector<RainflowCycle>& cycles,
    float k,
    float m,
    float ultimateStrength) {

    float sum = 0.0f;
    for (const auto& cycle : cycles) {
        float damagePerCycle = computeCycleDamage(
            cycle.range, cycle.mean, k, m, ultimateStrength);
        sum += damagePerCycle * cycle.count;
    }
    return sum;
}

float LifeAssessor::estimateDamageRate(
    const std::vector<RainflowCycle>& cycles,
    float k,
    float m,
    float ultimateStrength) {

    float damagePerSecond = minerSum(cycles, k, m, ultimateStrength);
    return damagePerSecond * 3600.0f;
}

float LifeAssessor::estimateRemainingLife(
    float cumulativeDamage,
    float currentDamageRate) {

    if (currentDamageRate <= 0) return expectedLifeHours_;

    float remainingDamage = 1.0f - cumulativeDamage;
    if (remainingDamage <= 0) return 0.0f;

    return remainingDamage / currentDamageRate;
}

BladeStress LifeAssessor::computeStress(
    const std::vector<float>& vibrationSignal,
    float cavitationIntensity,
    uint64_t timestamp,
    uint8_t turbineId,
    uint8_t bladeId,
    float sensitivity,
    float amplificationFactor) {

    std::lock_guard<std::mutex> lock(mutex_);

    BladeStress stress{};
    stress.timestamp = timestamp;
    stress.turbine_id = turbineId;
    stress.blade_id = bladeId;

    auto stressHistory = buildStressTimeHistory(
        vibrationSignal, cavitationIntensity, sensitivity, amplificationFactor);

    float sum = 0.0f;
    float maxVal = std::numeric_limits<float>::lowest();
    float minVal = std::numeric_limits<float>::max();

    for (float s : stressHistory) {
        sum += s;
        maxVal = std::max(maxVal, s);
        minVal = std::min(minVal, s);
    }

    stress.mean_stress = stressHistory.empty() ? 0.0f : sum / stressHistory.size();
    stress.max_stress = maxVal;
    stress.min_stress = minVal;
    stress.stress_amplitude = (maxVal - minVal) / 2.0f;
    stress.combined_stress = stress.mean_stress + stress.stress_amplitude;

    float vibrationRms = 0.0f;
    for (float v : vibrationSignal) {
        vibrationRms += v * v;
    }
    vibrationRms = std::sqrt(vibrationRms / std::max(size_t(1), vibrationSignal.size()));

    stress.vibration_stress = vibrationRms * sensitivity * amplificationFactor;
    stress.cavitation_stress = cavitationIntensity * 100.0f;

    auto cycles = rainflowCounting(stressHistory);
    stress.stress_cycles = 0;
    for (const auto& c : cycles) {
        stress.stress_cycles += c.count;
    }

    stress.rainflow_cycles.clear();
    for (const auto& c : cycles) {
        stress.rainflow_cycles.push_back(c.range);
        stress.rainflow_cycles.push_back(c.mean);
        stress.rainflow_cycles.push_back(static_cast<float>(c.count));
    }

    stressHistory_.push_back(stressHistory);
    if (stressHistory_.size() > 100) {
        stressHistory_.pop_front();
    }

    return stress;
}

LifeAssessment LifeAssessor::assessLife(
    const BladeStress& stressData,
    const CavitationState& cavitationState,
    uint8_t turbineId,
    uint8_t bladeId,
    float initialCumulativeDamage) {

    std::lock_guard<std::mutex> lock(mutex_);

    LifeAssessment assessment{};
    assessment.timestamp = currentTimestampMs();
    assessment.turbine_id = turbineId;
    assessment.blade_id = bladeId;
    assessment.assessment_method = "Rainflow + Miner Linear Damage";
    assessment.material_constant_k = materialProps_.k;
    assessment.material_constant_m = materialProps_.m;

    float& cumulativeDamageRef = cumulativeDamage_[turbineId][bladeId];

    if (cumulativeDamageRef == 0.0f && initialCumulativeDamage > 0) {
        cumulativeDamageRef = initialCumulativeDamage;
    }

    std::vector<RainflowCycle> cycles;
    for (size_t i = 0; i + 2 < stressData.rainflow_cycles.size(); i += 3) {
        cycles.push_back({
            stressData.rainflow_cycles[i],
            stressData.rainflow_cycles[i + 1],
            static_cast<uint32_t>(stressData.rainflow_cycles[i + 2])
        });
    }

    float fatigueDamage = minerSum(
        cycles,
        materialProps_.k,
        materialProps_.m,
        materialProps_.ultimateTensileStrength
    );

    float cavitationDamage = cavitationState.cavitation_intensity * 0.001f;

    assessment.fatigue_damage = fatigueDamage;
    assessment.cavitation_damage = cavitationDamage;

    cumulativeDamageRef += fatigueDamage + cavitationDamage;
    assessment.cumulative_damage = cumulativeDamageRef;
    assessment.miner_sum = cumulativeDamageRef;

    assessment.stress_range = stressData.max_stress - stressData.min_stress;
    assessment.cycle_count = stressData.stress_cycles;

    float damageRate = estimateDamageRate(
        cycles,
        materialProps_.k,
        materialProps_.m,
        materialProps_.ultimateTensileStrength
    );

    assessment.remaining_life_hours = estimateRemainingLife(
        cumulativeDamageRef,
        damageRate
    );
    assessment.remaining_life_days = assessment.remaining_life_hours / 24.0f;

    return assessment;
}

}
