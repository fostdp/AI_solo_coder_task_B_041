#include "../include/turbine_mpc_controller.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <numeric>

namespace turbine_monitor {

TurbineEfficiencyModel::TurbineEfficiencyModel() {
    head_axis_.resize(GRID_SIZE);
    flow_axis_.resize(GRID_SIZE);
    gv_axis_.resize(GRID_SIZE);
    hill_data_.resize(GRID_SIZE,
        std::vector<std::vector<float>>(GRID_SIZE,
            std::vector<float>(GRID_SIZE, 0.0f)));

    const float head_min = 80.0f;
    const float head_max = 160.0f;
    const float flow_min = 300.0f;
    const float flow_max = 900.0f;
    const float gv_min = 15.0f;
    const float gv_max = 85.0f;

    for (size_t i = 0; i < GRID_SIZE; ++i) {
        head_axis_[i] = head_min + (head_max - head_min) * i / (GRID_SIZE - 1);
        flow_axis_[i] = flow_min + (flow_max - flow_min) * i / (GRID_SIZE - 1);
        gv_axis_[i] = gv_min + (gv_max - gv_min) * i / (GRID_SIZE - 1);
    }

    const float head_opt = 120.0f;
    const float flow_opt = 620.0f;
    const float gv_opt = 52.0f;
    const float eff_max = 0.965f;

    for (size_t ih = 0; ih < GRID_SIZE; ++ih) {
        float h = head_axis_[ih];
        for (size_t iq = 0; iq < GRID_SIZE; ++iq) {
            float q = flow_axis_[iq];
            for (size_t ig = 0; ig < GRID_SIZE; ++ig) {
                float g = gv_axis_[ig];
                float dh = (h - head_opt) / 40.0f;
                float dq = (q - flow_opt) / 300.0f;
                float dg = (g - gv_opt) / 35.0f;
                float hill = eff_max * std::exp(-(dh*dh*1.5f + dq*dq*1.2f + dg*dg*1.8f));
                float gv_factor = 1.0f - 0.15f * std::abs(dg) - 0.1f * dg*dg;
                hill *= std::max(0.65f, gv_factor);
                hill_data_[ih][iq][ig] = std::max(0.60f, std::min(0.97f, hill));
            }
        }
    }
}

float TurbineEfficiencyModel::bilinearInterpolate(float h_norm, float q_norm, float gv_norm) {
    h_norm = std::max(0.0f, std::min(0.9999f, h_norm));
    q_norm = std::max(0.0f, std::min(0.9999f, q_norm));
    gv_norm = std::max(0.0f, std::min(0.9999f, gv_norm));

    float hf = h_norm * (GRID_SIZE - 1);
    float qf = q_norm * (GRID_SIZE - 1);
    float gf = gv_norm * (GRID_SIZE - 1);

    int ih0 = static_cast<int>(std::floor(hf));
    int iq0 = static_cast<int>(std::floor(qf));
    int ig0 = static_cast<int>(std::floor(gf));
    int ih1 = std::min(ih0 + 1, static_cast<int>(GRID_SIZE) - 1);
    int iq1 = std::min(iq0 + 1, static_cast<int>(GRID_SIZE) - 1);
    int ig1 = std::min(ig0 + 1, static_cast<int>(GRID_SIZE) - 1);

    float ah = hf - ih0;
    float aq = qf - iq0;
    float ag = gf - ig0;

    float c000 = hill_data_[ih0][iq0][ig0];
    float c001 = hill_data_[ih0][iq0][ig1];
    float c010 = hill_data_[ih0][iq1][ig0];
    float c011 = hill_data_[ih0][iq1][ig1];
    float c100 = hill_data_[ih1][iq0][ig0];
    float c101 = hill_data_[ih1][iq0][ig1];
    float c110 = hill_data_[ih1][iq1][ig0];
    float c111 = hill_data_[ih1][iq1][ig1];

    float c00 = c000 * (1-ag) + c001 * ag;
    float c01 = c010 * (1-ag) + c011 * ag;
    float c10 = c100 * (1-ag) + c101 * ag;
    float c11 = c110 * (1-ag) + c111 * ag;

    float c0 = c00 * (1-aq) + c01 * aq;
    float c1 = c10 * (1-aq) + c11 * aq;

    return c0 * (1-ah) + c1 * ah;
}

float TurbineEfficiencyModel::getEfficiency(float head, float flow, float guide_vane) {
    const float h_min = head_axis_.front();
    const float h_max = head_axis_.back();
    const float q_min = flow_axis_.front();
    const float q_max = flow_axis_.back();
    const float g_min = gv_axis_.front();
    const float g_max = gv_axis_.back();

    float h_norm = (head - h_min) / (h_max - h_min);
    float q_norm = (flow - q_min) / (q_max - q_min);
    float g_norm = (guide_vane - g_min) / (g_max - g_min);

    return bilinearInterpolate(h_norm, q_norm, g_norm);
}

CavitationRiskModel::CavitationRiskModel()
    : w_intensity_(0.5f), w_load_(0.3f), w_head_(0.2f) {}

float CavitationRiskModel::normalizeIntensity(float intensity) {
    return std::max(0.0f, std::min(1.0f, intensity));
}

float CavitationRiskModel::normalizeLoad(float load_ratio) {
    float x = std::abs(load_ratio - 0.75f) * 3.0f;
    return std::max(0.0f, std::min(1.0f, x));
}

float CavitationRiskModel::normalizeHeadDeviation(float head_dev) {
    float x = std::abs(head_dev) / 30.0f;
    return std::max(0.0f, std::min(1.0f, x));
}

float CavitationRiskModel::evaluateRisk(float cav_intensity, float load_ratio, float head_deviation) {
    float n_int = normalizeIntensity(cav_intensity);
    float n_load = normalizeLoad(load_ratio);
    float n_head = normalizeHeadDeviation(head_deviation);

    float risk = w_intensity_ * n_int + w_load_ * n_load + w_head_ * n_head;
    risk += n_int * n_load * 0.15f;

    return std::max(0.0f, std::min(1.0f, risk));
}

ModelPredictiveController::ModelPredictiveController(
    uint32_t prediction_horizon,
    uint32_t control_horizon,
    const MPCWeights& weights,
    float sample_time)
    : Np_(prediction_horizon), Nc_(control_horizon), dt_(sample_time),
      weights_(weights),
      max_iterations_(50), tolerance_(1e-4f), step_size_(0.01f) {

    constraints_.guide_vane_min = 15.0f;
    constraints_.guide_vane_max = 85.0f;
    constraints_.guide_vane_rate_max = 5.0f;
    constraints_.power_min = 100.0f;
    constraints_.power_max = 750.0f;
    constraints_.power_rate_max = 20.0f;
    constraints_.cav_risk_max = 0.8f;

    target_.target_power = 500.0f;
    target_.target_cav_risk = 0.2f;

    buildStateSpaceMatrices();
    discretize();
}

void ModelPredictiveController::setConstraints(const MPCConstraints& constraints) {
    constraints_ = constraints;
}

void ModelPredictiveController::setWeights(const MPCWeights& weights) {
    weights_ = weights;
}

void ModelPredictiveController::buildStateSpaceMatrices() {
    for (auto& row : A_cont_) {
        for (auto& v : row) v = 0.0f;
    }
    for (auto& row : B_cont_) {
        for (auto& v : row) v = 0.0f;
    }

    const float tau_gv = 0.8f;
    const float tau_p = 1.2f;
    const float tau_h = 2.0f;
    const float tau_q = 1.5f;
    const float tau_cav = 0.5f;

    A_cont_[0][0] = -1.0f / tau_gv;
    A_cont_[1][1] = -1.0f / tau_p;
    A_cont_[2][2] = -1.0f / tau_h;
    A_cont_[3][3] = -1.0f / tau_q;
    A_cont_[4][4] = -1.0f / tau_cav;

    A_cont_[1][0] = 3.5f;
    A_cont_[1][2] = 2.0f;
    A_cont_[1][3] = 1.2f;
    A_cont_[2][0] = 1.2f;
    A_cont_[2][3] = -0.8f;
    A_cont_[3][0] = 4.5f;
    A_cont_[3][2] = 0.6f;
    A_cont_[4][1] = 0.15f;
    A_cont_[4][2] = 0.005f;

    B_cont_[0][0] = 1.0f;
    B_cont_[1][1] = 1.0f;
    B_cont_[2][1] = 0.05f;
    B_cont_[3][0] = 0.8f;
    B_cont_[4][0] = 0.08f;
}

void ModelPredictiveController::discretize() {
    for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
        for (size_t j = 0; j < MPC_STATE_DIM; ++j) {
            A_disc_[i][j] = 0.0f;
        }
        A_disc_[i][i] = 1.0f;
    }
    for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
        for (size_t j = 0; j < MPC_INPUT_DIM; ++j) {
            B_disc_[i][j] = 0.0f;
        }
    }

    const size_t MAX_TAYLOR = 50;
    for (size_t k = 1; k <= MAX_TAYLOR; ++k) {
        float fact = 1.0f;
        for (size_t i = 2; i <= k; ++i) fact *= static_cast<float>(i);
        float pow_dt = std::pow(dt_, static_cast<int>(k)) / fact;

        std::array<std::array<float, MPC_STATE_DIM>, MPC_STATE_DIM> A_power{};
        std::array<std::array<float, MPC_STATE_DIM>, MPC_STATE_DIM> cur{};
        for (size_t i = 0; i < MPC_STATE_DIM; ++i) cur[i][i] = 1.0f;

        for (size_t exp = 0; exp < k; ++exp) {
            for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
                for (size_t j = 0; j < MPC_STATE_DIM; ++j) {
                    A_power[i][j] = 0.0f;
                    for (size_t l = 0; l < MPC_STATE_DIM; ++l) {
                        A_power[i][j] += cur[i][l] * A_cont_[l][j];
                    }
                }
            }
            cur = A_power;
        }

        for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
            for (size_t j = 0; j < MPC_STATE_DIM; ++j) {
                A_disc_[i][j] += pow_dt * cur[i][j];
            }
        }
    }

    std::array<std::array<float, MPC_STATE_DIM>, MPC_STATE_DIM> I_minus_A{};
    for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
        for (size_t j = 0; j < MPC_STATE_DIM; ++j) {
            I_minus_A[i][j] = (i == j ? 1.0f : 0.0f) - A_disc_[i][j];
        }
    }

    std::array<std::array<float, MPC_STATE_DIM>, MPC_STATE_DIM> inv_IminusA{};
    for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
        inv_IminusA[i][i] = 1.0f;
    }
    float max_abs = 0.0f;
    for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
        for (size_t j = 0; j < MPC_STATE_DIM; ++j) {
            max_abs = std::max(max_abs, std::abs(I_minus_A[i][j]));
        }
    }
    if (max_abs > 0.0f) {
        float alpha = 1.0f / (max_abs * max_abs * 2.0f);
        std::array<std::array<float, MPC_STATE_DIM>, MPC_STATE_DIM> B_mat{};
        for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
            for (size_t j = 0; j < MPC_STATE_DIM; ++j) {
                B_mat[i][j] = (i == j ? 1.0f : 0.0f) - alpha * I_minus_A[i][j];
            }
        }
        for (int iter = 0; iter < 100; ++iter) {
            std::array<std::array<float, MPC_STATE_DIM>, MPC_STATE_DIM> new_inv{};
            for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
                for (size_t j = 0; j < MPC_STATE_DIM; ++j) {
                    float sum = 0.0f;
                    for (size_t l = 0; l < MPC_STATE_DIM; ++l) {
                        sum += B_mat[i][l] * inv_IminusA[l][j];
                    }
                    new_inv[i][j] = sum;
                }
            }
            inv_IminusA = new_inv;
        }
    }

    for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
        for (size_t j = 0; j < MPC_INPUT_DIM; ++j) {
            float sum = 0.0f;
            for (size_t l = 0; l < MPC_STATE_DIM; ++l) {
                sum += inv_IminusA[i][l] * B_cont_[l][j];
            }
            B_disc_[i][j] = sum * dt_;
        }
    }
}

void ModelPredictiveController::predict(
    const MPCState& current_state,
    const std::vector<MPCInput>& control_sequence,
    std::vector<MPCState>& predicted_states) {

    predicted_states.clear();
    predicted_states.reserve(Np_ + 1);
    predicted_states.push_back(current_state);

    MPCState x = current_state;

    for (size_t k = 0; k < Np_; ++k) {
        const MPCInput& u = (k < control_sequence.size())
            ? control_sequence[k]
            : control_sequence.back();

        auto x_arr = x.toArray();
        auto u_arr = u.toArray();

        std::array<float, MPC_STATE_DIM> xdot_cont{};
        for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
            float sum = 0.0f;
            for (size_t j = 0; j < MPC_STATE_DIM; ++j) {
                sum += A_cont_[i][j] * x_arr[j];
            }
            for (size_t j = 0; j < MPC_INPUT_DIM; ++j) {
                sum += B_cont_[i][j] * u_arr[j];
            }
            xdot_cont[i] = sum;
        }

        std::array<float, MPC_STATE_DIM> x_mid{};
        for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
            x_mid[i] = x_arr[i] + 0.5f * dt_ * xdot_cont[i];
        }

        std::array<float, MPC_STATE_DIM> xdot_mid{};
        for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
            float sum = 0.0f;
            for (size_t j = 0; j < MPC_STATE_DIM; ++j) {
                sum += A_cont_[i][j] * x_mid[j];
            }
            for (size_t j = 0; j < MPC_INPUT_DIM; ++j) {
                sum += B_cont_[i][j] * u_arr[j];
            }
            xdot_mid[i] = sum;
        }

        std::array<float, MPC_STATE_DIM> x_new{};
        for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
            x_new[i] = x_arr[i] + dt_ * xdot_mid[i];
        }

        MPCState next_state = MPCState::fromArray(x_new);
        predicted_states.push_back(next_state);
        x = next_state;
    }
}

float ModelPredictiveController::computeCost(
    const std::vector<MPCInput>& u_sequence,
    const MPCState& x0) {

    std::vector<MPCState> states;
    predict(x0, u_sequence, states);

    float cost = 0.0f;

    for (size_t k = 1; k <= Np_; ++k) {
        const auto& x = states[k];

        float eff = efficiency_model_.getEfficiency(x.head, x.flow, x.guide_vane);
        float load_ratio = x.power / (constraints_.power_max - constraints_.power_min + 1e-6f);
        float head_dev = x.head - 120.0f;
        float cav = cavitation_model_.evaluateRisk(x.cav_risk, load_ratio, head_dev);
        float power_error = x.power - target_.target_power;

        cost += weights_.W_eff * (1.0f - eff);
        cost += weights_.W_cav * cav * cav;
        cost += weights_.W_power * power_error * power_error / 1e4f;

        if (x.cav_risk > constraints_.cav_risk_max) {
            cost += 100.0f * (x.cav_risk - constraints_.cav_risk_max);
        }
    }

    for (size_t k = 0; k < std::min(u_sequence.size(), Nc_); ++k) {
        const auto& u = u_sequence[k];
        cost += weights_.W_du * (u.guide_vane_rate * u.guide_vane_rate
                               + u.power_rate * u.power_rate);
    }

    return cost;
}

void ModelPredictiveController::computeGradient(
    const std::vector<MPCInput>& u_sequence,
    const MPCState& x0,
    std::vector<MPCInput>& gradient) {

    const float eps = 1e-4f;
    size_t seq_len = u_sequence.size();
    gradient.assign(seq_len, MPCInput{0.0f, 0.0f});

    float base_cost = computeCost(u_sequence, x0);

    for (size_t k = 0; k < seq_len; ++k) {
        auto u_plus = u_sequence;
        u_plus[k].guide_vane_rate += eps;
        float cost_plus = computeCost(u_plus, x0);
        gradient[k].guide_vane_rate = (cost_plus - base_cost) / eps;

        u_plus = u_sequence;
        u_plus[k].power_rate += eps;
        cost_plus = computeCost(u_plus, x0);
        gradient[k].power_rate = (cost_plus - base_cost) / eps;
    }
}

void ModelPredictiveController::clampInput(MPCInput& u, const MPCState& prev_state) {
    u.guide_vane_rate = std::max(-constraints_.guide_vane_rate_max,
                          std::min(constraints_.guide_vane_rate_max, u.guide_vane_rate));
    u.power_rate = std::max(-constraints_.power_rate_max,
                     std::min(constraints_.power_rate_max, u.power_rate));

    float next_gv = prev_state.guide_vane + u.guide_vane_rate * dt_;
    if (next_gv < constraints_.guide_vane_min) {
        u.guide_vane_rate = (constraints_.guide_vane_min - prev_state.guide_vane) / dt_;
    }
    if (next_gv > constraints_.guide_vane_max) {
        u.guide_vane_rate = (constraints_.guide_vane_max - prev_state.guide_vane) / dt_;
    }

    float next_power = prev_state.power + u.power_rate * dt_;
    if (next_power < constraints_.power_min) {
        u.power_rate = (constraints_.power_min - prev_state.power) / dt_;
    }
    if (next_power > constraints_.power_max) {
        u.power_rate = (constraints_.power_max - prev_state.power) / dt_;
    }
}

void ModelPredictiveController::projectConstraints(
    std::vector<MPCInput>& u_sequence,
    const MPCState& x0) {

    std::vector<MPCState> states;
    predict(x0, u_sequence, states);

    for (size_t k = 0; k < u_sequence.size(); ++k) {
        const auto& prev_state = (k == 0) ? x0 : states[k];
        clampInput(u_sequence[k], prev_state);
    }
}

bool ModelPredictiveController::checkStateConstraints(const MPCState& state) {
    if (state.guide_vane < constraints_.guide_vane_min - 1e-3f) return false;
    if (state.guide_vane > constraints_.guide_vane_max + 1e-3f) return false;
    if (state.power < constraints_.power_min - 1e-3f) return false;
    if (state.power > constraints_.power_max + 1e-3f) return false;
    if (state.cav_risk > constraints_.cav_risk_max + 1e-3f) return false;
    return true;
}

MPCSolution ModelPredictiveController::solveQP(
    const MPCState& current_state,
    const MPCInput& last_u) {

    MPCSolution solution;
    solution.iterations = 0;
    solution.converged = false;

    std::vector<MPCInput> u_sequence(Nc_, last_u);
    projectConstraints(u_sequence, current_state);

    float prev_cost = computeCost(u_sequence, current_state);
    float best_cost = prev_cost;
    std::vector<MPCInput> best_u = u_sequence;

    std::vector<MPCInput> gradient;
    float cur_step = step_size_;

    for (int iter = 0; iter < max_iterations_; ++iter) {
        solution.iterations = iter + 1;

        computeGradient(u_sequence, current_state, gradient);

        float grad_norm = 0.0f;
        for (const auto& g : gradient) {
            grad_norm += g.guide_vane_rate * g.guide_vane_rate;
            grad_norm += g.power_rate * g.power_rate;
        }
        grad_norm = std::sqrt(grad_norm);

        if (grad_norm < tolerance_) {
            solution.converged = true;
            break;
        }

        std::vector<MPCInput> u_new = u_sequence;
        float step_scaled = cur_step / std::max(1e-6f, grad_norm);
        for (size_t k = 0; k < Nc_; ++k) {
            u_new[k].guide_vane_rate -= step_scaled * gradient[k].guide_vane_rate;
            u_new[k].power_rate -= step_scaled * gradient[k].power_rate;
        }

        projectConstraints(u_new, current_state);
        float new_cost = computeCost(u_new, current_state);

        if (new_cost < best_cost) {
            best_cost = new_cost;
            best_u = u_new;
            cur_step *= 1.2f;
        } else {
            cur_step *= 0.5f;
        }

        u_sequence = best_u;
        float cost_change = std::abs(new_cost - prev_cost);
        prev_cost = new_cost;

        if (cost_change < tolerance_ && iter > 5) {
            solution.converged = true;
            break;
        }
    }

    solution.optimal_u0 = {best_u[0].guide_vane_rate, best_u[0].power_rate};
    predict(current_state, best_u, solution.predicted_states);
    solution.cost_value = best_cost;

    return solution;
}

TurbineControlCommand ModelPredictiveController::solve(
    const MPCState& current_state,
    const MPCInput& last_u,
    uint8_t turbine_id) {

    MPCSolution mpc_sol = solveQP(current_state, last_u);

    TurbineControlCommand cmd{};
    cmd.timestamp = currentTimestampMs();
    cmd.turbine_id = turbine_id;
    cmd.control_mode = ControlMode::MPC_OPTIMAL;
    cmd.cavitation_avoidance_enabled = true;
    cmd.guide_vane_opening_deg = current_state.guide_vane
        + mpc_sol.optimal_u0[0] * dt_;
    cmd.target_power_mw = current_state.power
        + mpc_sol.optimal_u0[1] * dt_;
    cmd.current_head_m = current_state.head;
    cmd.current_flow_m3s = current_state.flow;
    cmd.mpc_cost_value = mpc_sol.cost_value;

    if (!mpc_sol.predicted_states.empty()) {
        size_t pred_idx = std::min(size_t(5), mpc_sol.predicted_states.size() - 1);
        const auto& ps = mpc_sol.predicted_states[pred_idx];
        cmd.predicted_efficiency = efficiency_model_.getEfficiency(
            ps.head, ps.flow, ps.guide_vane);
        float load_ratio = ps.power / (constraints_.power_max - constraints_.power_min + 1e-6f);
        float head_dev = ps.head - 120.0f;
        cmd.predicted_cavitation_risk = cavitation_model_.evaluateRisk(
            ps.cav_risk, load_ratio, head_dev);
    }

    cmd.control_signals = mpc_sol.optimal_u0;
    cmd.horizon_states.reserve(mpc_sol.predicted_states.size() * MPC_STATE_DIM);
    for (const auto& s : mpc_sol.predicted_states) {
        cmd.horizon_states.push_back(s.guide_vane);
        cmd.horizon_states.push_back(s.power);
        cmd.horizon_states.push_back(s.head);
        cmd.horizon_states.push_back(s.flow);
        cmd.horizon_states.push_back(s.cav_risk);
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    oss << "MPC: GV=" << cmd.guide_vane_opening_deg << "deg";
    oss << ", P=" << cmd.target_power_mw << "MW";
    oss << ", iter=" << mpc_sol.iterations;
    oss << (mpc_sol.converged ? "(ok)" : "(lim)");
    cmd.control_action_desc = oss.str();

    return cmd;
}

}
