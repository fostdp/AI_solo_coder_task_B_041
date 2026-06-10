#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include "data_structures.h"
#include "config.h"

namespace turbine_monitor {

constexpr size_t MPC_STATE_DIM = 5;
constexpr size_t MPC_INPUT_DIM = 2;

struct MPCState {
    float guide_vane;
    float power;
    float head;
    float flow;
    float cav_risk;

    std::array<float, MPC_STATE_DIM> toArray() const {
        return {guide_vane, power, head, flow, cav_risk};
    }

    static MPCState fromArray(const std::array<float, MPC_STATE_DIM>& arr) {
        return {arr[0], arr[1], arr[2], arr[3], arr[4]};
    }
};

struct MPCInput {
    float guide_vane_rate;
    float power_rate;

    std::array<float, MPC_INPUT_DIM> toArray() const {
        return {guide_vane_rate, power_rate};
    }
};

struct MPCWeights {
    float W_eff;
    float W_cav;
    float W_power;
    float W_du;
};

struct MPCConstraints {
    float guide_vane_min;
    float guide_vane_max;
    float guide_vane_rate_max;
    float power_min;
    float power_max;
    float power_rate_max;
    float cav_risk_max;
};

struct MPCTarget {
    float target_power;
    float target_cav_risk;
};

struct MPCSolution {
    std::vector<float> optimal_u0;
    std::vector<MPCState> predicted_states;
    float cost_value;
    int iterations;
    bool converged;
};

class TurbineEfficiencyModel {
public:
    TurbineEfficiencyModel();

    float getEfficiency(float head, float flow, float guide_vane);

    void setGridData(const std::vector<std::vector<std::vector<float>>>& data) {
        hill_data_ = data; }

private:
    float bilinearInterpolate(float h_norm, float q_norm, float gv_norm);

    std::vector<float> head_axis_;
    std::vector<float> flow_axis_;
    std::vector<float> gv_axis_;
    std::vector<std::vector<std::vector<float>>> hill_data_;

    static constexpr size_t GRID_SIZE = 10;
};

class CavitationRiskModel {
public:
    CavitationRiskModel();

    float evaluateRisk(float cav_intensity, float load_ratio, float head_deviation);

    void setParams(float w_intensity, float w_load, float w_head) {
        w_intensity_ = w_intensity; w_load_ = w_load; w_head_ = w_head; }

private:
    float normalizeIntensity(float intensity);
    float normalizeLoad(float load_ratio);
    float normalizeHeadDeviation(float head_dev);

    float w_intensity_;
    float w_load_;
    float w_head_;
};

class ModelPredictiveController {
public:
    ModelPredictiveController(
        uint32_t prediction_horizon = 20,
        uint32_t control_horizon = 5,
        const MPCWeights& weights = {1.0f, 3.0f, 2.0f, 0.5f},
        float sample_time = 0.1f);

    void setConstraints(const MPCConstraints& constraints);
    void setWeights(const MPCWeights& weights);
    void setTarget(const MPCTarget& target) { target_ = target; }

    void predict(const MPCState& current_state,
                  const std::vector<MPCInput>& control_sequence,
                  std::vector<MPCState>& predicted_states);

    MPCTarget target_;

    MPCSolution solveQP(const MPCState& current_state,
                        const MPCInput& last_u);

    TurbineControlCommand solve(const MPCState& current_state,
                              const MPCInput& last_u,
                              uint8_t turbine_id);

    TurbineEfficiencyModel& efficiencyModel() { return efficiency_model_; }
    CavitationRiskModel& cavitationModel() { return cavitation_model_; }

    void setMaxIterations(int max_iter) { max_iterations_ = max_iter; }
    void setTolerance(float tol) { tolerance_ = tol; }

private:
    void buildStateSpaceMatrices();
    void discretize();
    float computeCost(const std::vector<MPCInput>& u_sequence,
                   const MPCState& x0);
    void computeGradient(const std::vector<MPCInput>& u_sequence,
                       const MPCState& x0,
                       std::vector<MPCInput>& gradient);
    void projectConstraints(std::vector<MPCInput>& u_sequence,
                            const MPCState& x0);
    bool checkStateConstraints(const MPCState& state);
    void clampInput(MPCInput& u, const MPCState& prev_state);

    uint32_t Np_;
    uint32_t Nc_;
    float dt_;
    MPCWeights weights_;
    MPCConstraints constraints_;

    std::array<std::array<float, MPC_STATE_DIM>, MPC_STATE_DIM> A_cont_;
    std::array<std::array<float, MPC_INPUT_DIM>, MPC_STATE_DIM> B_cont_;
    std::array<std::array<float, MPC_STATE_DIM>, MPC_STATE_DIM> A_disc_;
    std::array<std::array<float, MPC_INPUT_DIM>, MPC_STATE_DIM> B_disc_;

    int max_iterations_;
    float tolerance_;
    float step_size_;

    TurbineEfficiencyModel efficiency_model_;
    CavitationRiskModel cavitation_model_;
};

}
