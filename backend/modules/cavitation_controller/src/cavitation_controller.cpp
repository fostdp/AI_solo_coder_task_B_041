#include "cavitation_controller.h"
#include <cstring>

namespace cav_ctrl {

static constexpr float H0 = 80.0f;
static constexpr float Q0 = 200.0f;
static constexpr float GV0 = 50.0f;
static constexpr float ETA_MAX = 0.965f;
static constexpr float R_LOW = 0.30f;
static constexpr float R_HIGH = 0.85f;

TurbineEfficiencyModel::TurbineEfficiencyModel() {
    hill_data_.resize(GRID_SIZE);
    for (auto& h : hill_data_) {
        h.resize(GRID_SIZE);
        for (auto& g : h) g.resize(GRID_SIZE, 0.0f);
    }
    head_axis_.resize(GRID_SIZE);
    flow_axis_.resize(GRID_SIZE);
    gv_axis_.resize(GRID_SIZE);
    for (size_t i = 0; i < GRID_SIZE; ++i) {
        head_axis_[i] = H0 * (0.85f + 0.30f * i / (GRID_SIZE - 1));
        flow_axis_[i] = Q0 * (0.50f + 1.00f * i / (GRID_SIZE - 1));
        gv_axis_[i]   = 15.0f + 70.0f * i / (GRID_SIZE - 1);
    }
    for (size_t hi = 0; hi < GRID_SIZE; ++hi) {
        for (size_t qi = 0; qi < GRID_SIZE; ++qi) {
            for (size_t gi = 0; gi < GRID_SIZE; ++gi) {
                float h_norm = (head_axis_[hi] - H0) / (H0 * 0.15f);
                float q_norm = (flow_axis_[qi] - Q0) / (Q0 * 0.50f);
                float gv_norm = (gv_axis_[gi] - GV0) / (GV0 * 0.70f);
                float bell = std::exp(-0.5f * (h_norm * h_norm + q_norm * q_norm));
                float gv_factor = std::exp(-gv_norm * gv_norm * 2.0f);
                hill_data_[hi][qi][gi] = ETA_MAX * bell * gv_factor + 0.70f * (1.0f - bell);
                hill_data_[hi][qi][gi] = std::max(0.65f, std::min(0.97f, hill_data_[hi][qi][gi]));
            }
        }
    }
}

float TurbineEfficiencyModel::bilinearInterpolate(float h_norm, float q_norm, float gv_norm) {
    auto clamp_idx = [](float x, size_t n) -> size_t {
        if (x < 0) return 0;
        if (x > static_cast<float>(n - 1)) return n - 1;
        return static_cast<size_t>(x);
    };
    size_t gi0 = clamp_idx(gv_norm * (GRID_SIZE - 1), GRID_SIZE);
    size_t gi1 = std::min(gi0 + 1, GRID_SIZE - 1);
    size_t hi0 = clamp_idx(h_norm * (GRID_SIZE - 1), GRID_SIZE);
    size_t hi1 = std::min(hi0 + 1, GRID_SIZE - 1);
    size_t qi0 = clamp_idx(q_norm * (GRID_SIZE - 1), GRID_SIZE);
    size_t qi1 = std::min(qi0 + 1, GRID_SIZE - 1);
    float v000 = hill_data_[hi0][qi0][gi0];
    float v100 = hill_data_[hi1][qi0][gi0];
    float v010 = hill_data_[hi0][qi1][gi0];
    float v110 = hill_data_[hi1][qi1][gi0];
    float v001 = hill_data_[hi0][qi0][gi1];
    float v101 = hill_data_[hi1][qi0][gi1];
    float v011 = hill_data_[hi0][qi1][gi1];
    float v111 = hill_data_[hi1][qi1][gi1];
    float th = h_norm * (GRID_SIZE - 1) - hi0;
    float tq = q_norm * (GRID_SIZE - 1) - qi0;
    float tg = gv_norm * (GRID_SIZE - 1) - gi0;
    float c00 = v000 * (1 - th) + v100 * th;
    float c10 = v010 * (1 - th) + v110 * th;
    float c01 = v001 * (1 - th) + v101 * th;
    float c11 = v011 * (1 - th) + v111 * th;
    float c0 = c00 * (1 - tq) + c10 * tq;
    float c1 = c01 * (1 - tq) + c11 * tq;
    return c0 * (1 - tg) + c1 * tg;
}

float TurbineEfficiencyModel::getEfficiency(float head, float flow, float guide_vane) {
    float h_norm = std::max(0.0f, std::min(1.0f, (head - H0 * 0.85f) / (H0 * 0.30f)));
    float q_norm = std::max(0.0f, std::min(1.0f, (flow - Q0 * 0.50f) / (Q0 * 0.50f)));
    float gv_norm = std::max(0.0f, std::min(1.0f, (guide_vane - 15.0f) / 70.0f));
    return bilinearInterpolate(h_norm, q_norm, gv_norm);
}

CavitationRiskModel::CavitationRiskModel() :
    w_intensity_(0.55f), w_load_(0.25f), w_head_(0.20f) {}

float CavitationRiskModel::normalizeIntensity(float intensity) {
    return std::max(0.0f, std::min(1.0f, intensity / 0.6f));
}

float CavitationRiskModel::normalizeLoad(float load_ratio) {
    float lr = std::max(0.2f, std::min(1.5f, load_ratio));
    return lr < 0.5f ? (0.5f - lr) * 2.0f : (lr - 0.9f) * 2.5f;
}

float CavitationRiskModel::normalizeHeadDeviation(float head_dev) {
    return std::max(0.0f, std::min(1.0f, std::abs(head_dev) / 10.0f));
}

float CavitationRiskModel::evaluateRisk(float cav_intensity, float load_ratio, float head_deviation) {
    float r = w_intensity_ * normalizeIntensity(cav_intensity)
            + w_load_    * normalizeLoad(load_ratio)
            + w_head_    * normalizeHeadDeviation(head_deviation);
    return std::max(R_LOW, std::min(R_HIGH, R_LOW + (R_HIGH - R_LOW) * r));
}

ModelPredictiveController::ModelPredictiveController(
    uint32_t prediction_horizon, uint32_t control_horizon,
    const MPCWeights& weights, float sample_time)
    : Np_(prediction_horizon), Nc_(control_horizon), dt_(sample_time),
      weights_(weights), max_iterations_(50), tolerance_(1e-4f), step_size_(0.02f) {
    for (auto& row : A_cont_) for (auto& v : row) v = 0.0f;
    for (auto& row : B_cont_) for (auto& v : row) v = 0.0f;
    buildStateSpaceMatrices();
    discretize();
}

void ModelPredictiveController::buildStateSpaceMatrices() {
    A_cont_[0][0] = -0.25f; A_cont_[0][2] = 0.08f;
    A_cont_[1][1] = -0.15f; A_cont_[1][0] = 0.10f; A_cont_[1][2] = 0.05f;
    A_cont_[2][2] = -0.05f;
    A_cont_[3][3] = -0.10f; A_cont_[3][0] = 0.15f; A_cont_[3][2] = 0.10f;
    A_cont_[4][4] = -0.08f; A_cont_[4][3] = 0.12f;
    B_cont_[0][0] = 1.0f;
    B_cont_[1][1] = 1.0f;
    B_cont_[3][0] = 0.3f; B_cont_[3][1] = 0.2f;
    B_cont_[4][0] = 0.2f;
}

void ModelPredictiveController::discretize() {
    float dt = dt_;
    for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
        for (size_t j = 0; j < MPC_STATE_DIM; ++j) {
            A_disc_[i][j] = (i == j ? 1.0f : 0.0f) + A_cont_[i][j] * dt;
        }
        for (size_t j = 0; j < MPC_INPUT_DIM; ++j) {
            B_disc_[i][j] = B_cont_[i][j] * dt;
        }
    }
}

void ModelPredictiveController::setConstraints(const MPCConstraints& c) { constraints_ = c; }
void ModelPredictiveController::setWeights(const MPCWeights& w) { weights_ = w; }

void ModelPredictiveController::predict(const MPCState& current_state,
    const std::vector<MPCInput>& control_sequence,
    std::vector<MPCState>& predicted_states) {
    predicted_states.clear();
    predicted_states.reserve(Np_ + 1);
    predicted_states.push_back(current_state);
    MPCState x = current_state;
    for (size_t k = 0; k < Np_; ++k) {
        const MPCInput& u = k < control_sequence.size() ? control_sequence[k] : control_sequence.back();
        auto xa = x.toArray();
        auto ua = u.toArray();
        std::array<float, MPC_STATE_DIM> x_next{};
        for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
            x_next[i] = 0;
            for (size_t j = 0; j < MPC_STATE_DIM; ++j) x_next[i] += A_disc_[i][j] * xa[j];
            for (size_t j = 0; j < MPC_INPUT_DIM; ++j) x_next[i] += B_disc_[i][j] * ua[j];
        }
        x = MPCState::fromArray(x_next);
        predicted_states.push_back(x);
    }
}

bool ModelPredictiveController::checkStateConstraints(const MPCState& s) {
    return s.guide_vane >= constraints_.guide_vane_min &&
           s.guide_vane <= constraints_.guide_vane_max &&
           s.power >= constraints_.power_min && s.power <= constraints_.power_max &&
           s.cav_risk <= constraints_.cav_risk_max;
}

void ModelPredictiveController::clampInput(MPCInput& u, const MPCState& prev_state) {
    float max_dgv = constraints_.guide_vane_rate_max * dt_;
    float max_dp = constraints_.power_rate_max * dt_;
    u.guide_vane_rate = std::max(-max_dgv, std::min(max_dgv, u.guide_vane_rate));
    u.power_rate = std::max(-max_dp, std::min(max_dp, u.power_rate));
    float next_gv = prev_state.guide_vane + u.guide_vane_rate;
    if (next_gv < constraints_.guide_vane_min) u.guide_vane_rate = constraints_.guide_vane_min - prev_state.guide_vane;
    if (next_gv > constraints_.guide_vane_max) u.guide_vane_rate = constraints_.guide_vane_max - prev_state.guide_vane;
    float next_p = prev_state.power + u.power_rate;
    if (next_p < constraints_.power_min) u.power_rate = constraints_.power_min - prev_state.power;
    if (next_p > constraints_.power_max) u.power_rate = constraints_.power_max - prev_state.power;
}

float ModelPredictiveController::computeCost(const std::vector<MPCInput>& u_sequence, const MPCState& x0) {
    std::vector<MPCState> pred;
    predict(x0, u_sequence, pred);
    float cost = 0.0f;
    for (size_t k = 1; k <= Np_; ++k) {
        const MPCState& s = pred[k];
        float eta = efficiency_model_.getEfficiency(s.head, s.flow, s.guide_vane);
        float eff_dev = (1.0f - eta) * (1.0f - eta);
        float cav_pen = std::max(0.0f, s.cav_risk - target_.target_cav_risk);
        cav_pen *= cav_pen;
        float pow_dev = (s.power - target_.target_power) / constraints_.power_max;
        pow_dev *= pow_dev;
        cost += weights_.W_eff * eff_dev + weights_.W_cav * cav_pen + weights_.W_power * pow_dev;
    }
    for (size_t k = 0; k < Nc_ && k < u_sequence.size(); ++k) {
        if (k > 0) {
            float dgv = u_sequence[k].guide_vane_rate - u_sequence[k-1].guide_vane_rate;
            float dp  = u_sequence[k].power_rate - u_sequence[k-1].power_rate;
            cost += weights_.W_du * (dgv * dgv + dp * dp);
        }
    }
    return cost;
}

void ModelPredictiveController::computeGradient(const std::vector<MPCInput>& u_sequence,
    const MPCState& x0, std::vector<MPCInput>& gradient) {
    const float eps = 1e-4f;
    gradient.assign(u_sequence.size(), {0.0f, 0.0f});
    float cost0 = computeCost(u_sequence, x0);
    for (size_t k = 0; k < u_sequence.size(); ++k) {
        std::vector<MPCInput> up = u_sequence;
        up[k].guide_vane_rate += eps;
        gradient[k].guide_vane_rate = (computeCost(up, x0) - cost0) / eps;
        up = u_sequence;
        up[k].power_rate += eps;
        gradient[k].power_rate = (computeCost(up, x0) - cost0) / eps;
    }
}

void ModelPredictiveController::projectConstraints(std::vector<MPCInput>& u_sequence, const MPCState& x0) {
    MPCState s = x0;
    for (size_t k = 0; k < u_sequence.size(); ++k) {
        clampInput(u_sequence[k], s);
        auto xa = s.toArray();
        auto ua = u_sequence[k].toArray();
        std::array<float, MPC_STATE_DIM> x_next{};
        for (size_t i = 0; i < MPC_STATE_DIM; ++i) {
            for (size_t j = 0; j < MPC_STATE_DIM; ++j) x_next[i] += A_disc_[i][j] * xa[j];
            for (size_t j = 0; j < MPC_INPUT_DIM; ++j) x_next[i] += B_disc_[i][j] * ua[j];
        }
        s = MPCState::fromArray(x_next);
    }
}

MPCSolution ModelPredictiveController::solveQP(const MPCState& current_state, const MPCInput& last_u) {
    MPCSolution sol;
    std::vector<MPCInput> u_seq(Nc_, {0.0f, 0.0f});
    for (auto& u : u_seq) { u.guide_vane_rate = last_u.guide_vane_rate; u.power_rate = last_u.power_rate; }
    std::vector<MPCInput> grad;
    float prev_cost = computeCost(u_seq, current_state);
    sol.converged = false;
    for (sol.iterations = 0; sol.iterations < max_iterations_; ++sol.iterations) {
        computeGradient(u_seq, current_state, grad);
        for (size_t k = 0; k < u_seq.size(); ++k) {
            u_seq[k].guide_vane_rate -= step_size_ * grad[k].guide_vane_rate;
            u_seq[k].power_rate      -= step_size_ * grad[k].power_rate;
        }
        projectConstraints(u_seq, current_state);
        float cost = computeCost(u_seq, current_state);
        if (std::abs(cost - prev_cost) < tolerance_) {
            sol.converged = true;
            sol.cost_value = cost;
            break;
        }
        prev_cost = cost;
        sol.cost_value = cost;
    }
    sol.optimal_u0 = {u_seq[0].guide_vane_rate, u_seq[0].power_rate};
    predict(current_state, u_seq, sol.predicted_states);
    return sol;
}

float ModelPredictiveController::applyFeedForward(float gv_cmd, float gv_actual, float dgv_base) {
    if (!ff_cfg_.enabled) return dgv_base;
    float err = gv_cmd - gv_actual;
    if (std::abs(err) > ff_cfg_.threshold_deg) {
        float extra = ff_cfg_.gain * err;
        float max_step = ff_cfg_.max_step_gv_ff;
        dgv_base += extra;
        dgv_base = std::max(-max_step, std::min(max_step, dgv_base));
    }
    return dgv_base;
}

TurbineControlCommand ModelPredictiveController::solve(const MPCState& current_state,
    const MPCInput& last_u, uint8_t turbine_id) {
    auto sol = solveQP(current_state, last_u);
    MPCInput u = {sol.optimal_u0[0], sol.optimal_u0[1]};
    clampInput(u, current_state);
    float target_gv = current_state.guide_vane + u.guide_vane_rate;
    float target_p  = current_state.power + u.power_rate;
    float dgv_ff = applyFeedForward(target_gv, current_state.guide_vane, u.guide_vane_rate);
    target_gv = current_state.guide_vane + dgv_ff;
    TurbineControlCommand cmd;
    cmd.turbine_id = turbine_id;
    cmd.guide_vane_target = std::max(constraints_.guide_vane_min, std::min(constraints_.guide_vane_max, target_gv));
    cmd.power_target = std::max(constraints_.power_min, std::min(constraints_.power_max, target_p));
    cmd.control_mode = ControlMode::OPTIMIZE_EFFICIENCY;
    cmd.feedforward_applied = ff_cfg_.enabled && std::abs(target_gv - current_state.guide_vane - u.guide_vane_rate) > 1e-3f;
    cmd.timestamp_ms = 0;
    return cmd;
}

CavitationController::CavitationController(uint8_t turbine_id) : turbine_id_(turbine_id) {
    MPCConstraints c;
    c.guide_vane_min = 15.0f;
    c.guide_vane_max = 85.0f;
    c.guide_vane_rate_max = 50.0f;
    c.power_min = 100.0f;
    c.power_max = 750.0f;
    c.power_rate_max = 3000.0f;
    c.cav_risk_max = 0.5f;
    mpc_.setConstraints(c);
    FeedForwardConfig ff;
    ff.enabled = true;
    mpc_.setFeedForward(ff);
    latest_state_ = {52.0f, 500.0f, 80.0f, 200.0f, 0.35f};
    last_u_ = {0.0f, 0.0f};
    std::memset(&latest_cmd_, 0, sizeof(latest_cmd_));
    latest_cmd_.turbine_id = turbine_id_;
    latest_cmd_.guide_vane_target = 52.0f;
    latest_cmd_.power_target = 500.0f;
    latest_cmd_.control_mode = ControlMode::OPTIMIZE_EFFICIENCY;
}

CavitationController::~CavitationController() { stop(); }

bool CavitationController::init(const Config::ControllerConfig& cfg) {
    MPCConstraints c;
    c.guide_vane_min = cfg.guide_vane_min;
    c.guide_vane_max = cfg.guide_vane_max;
    c.guide_vane_rate_max = cfg.guide_vane_rate_max;
    c.power_min = cfg.power_min_mw;
    c.power_max = cfg.power_max_mw;
    c.power_rate_max = cfg.power_ramp_rate_mw_min;
    c.cav_risk_max = cfg.cavitation_risk_max;
    mpc_.setConstraints(c);
    FeedForwardConfig ff;
    ff.enabled = cfg.feedforward_enabled;
    ff.gain    = cfg.feedforward_gain;
    mpc_.setFeedForward(ff);
    return true;
}

void CavitationController::setTarget(const MPCTarget& target) {
    mpc_.setTarget(target);
}

void CavitationController::updateState(const MPCState& state) {
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        latest_state_ = state;
    }
    state_updated_.store(true);
    cv_.notify_one();
}

MPCState CavitationController::getLatestState() const {
    std::lock_guard<std::mutex> lk(state_mutex_);
    return latest_state_;
}

TurbineControlCommand CavitationController::getLatestCommand() const {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    return latest_cmd_;
}

void CavitationController::setFeedForwardEnabled(bool enabled) {
    FeedForwardConfig ff = mpc_.feedForwardConfig();
    ff.enabled = enabled;
    mpc_.setFeedForward(ff);
}

void CavitationController::start() {
    if (running_.load()) return;
    running_.store(true);
    worker_ = std::make_unique<std::thread>(&CavitationController::workerLoop, this);
}

void CavitationController::stop() {
    running_.store(false);
    cv_.notify_all();
    if (worker_ && worker_->joinable()) worker_->join();
    worker_.reset();
}

void CavitationController::workerLoop() {
    while (running_.load()) {
        std::unique_lock<std::mutex> lk(cv_mutex_);
        cv_.wait_for(lk, cycle_interval_, [this]() { return state_updated_.load() || !running_.load(); });
        if (!running_.load()) break;
        state_updated_.store(false);
        MPCState state;
        MPCInput u;
        {
            std::lock_guard<std::mutex> slk(state_mutex_);
            state = latest_state_;
            u = last_u_;
        }
        auto t0 = std::chrono::high_resolution_clock::now();
        TurbineControlCommand cmd = mpc_.solve(state, u, turbine_id_);
        auto t1 = std::chrono::high_resolution_clock::now();
        last_solve_ms_.store(static_cast<float>(std::chrono::duration<double, std::milli>(t1 - t0).count()));
        solve_count_.fetch_add(1);
        {
            std::lock_guard<std::mutex> clk(cmd_mutex_);
            latest_cmd_ = cmd;
        }
        last_u_ = {cmd.guide_vane_target - state.guide_vane, cmd.power_target - state.power};
        if (cmd_callback_) cmd_callback_(cmd);
    }
}

}
