#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <string>
#include <memory>
#include <mutex>
#include <deque>
#include <cmath>
#include <algorithm>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include "data_structures.h"
#include "config.h"

namespace plant_sched {

constexpr uint8_t  SCHED_UNITS   = 6;
constexpr uint8_t  SCHED_HOURS   = 24;
constexpr uint8_t  HILL_LOAD_PTS = 10;
constexpr uint8_t  HILL_HEAD_PTS = 5;
constexpr uint8_t  MIN_STABLE_HOURS = 6;

struct SchedulerConfig {
    uint8_t min_stable_hours = MIN_STABLE_HOURS;
    float   w_efficiency     = 1.0f;
    float   w_cavitation     = 3.0f;
    float   w_startup        = 0.5f;
    float   reserve_pct      = 5.0f;
    float   gap_tolerance    = 0.1f;
    uint64_t time_limit_ms   = 5000;
};

class UnitEfficiencyCurve {
public:
    UnitEfficiencyCurve();

    void setUnitId(uint8_t id) { unitId_ = id; }
    uint8_t unitId() const { return unitId_; }

    void setRatedPowerMW(float mw) { ratedPowerMW_ = mw; }
    float ratedPowerMW() const { return ratedPowerMW_; }

    void setRatedHeadM(float h) { ratedHeadM_ = h; }
    float ratedHeadM() const { return ratedHeadM_; }

    void setHillChart(const std::array<std::array<float, HILL_LOAD_PTS>, HILL_HEAD_PTS>& table);

    float efficiency(float powerMW, float headM) const;

    float cavitationPenalty(float powerMW, float headM) const;

    float startupCost(float operatingHours) const;

    const std::array<std::array<float, HILL_LOAD_PTS>, HILL_HEAD_PTS>& hillChart() const {
        return hillChart_;
    }

    const std::array<float, HILL_LOAD_PTS>& loadPoints() const { return loadPoints_; }
    const std::array<float, HILL_HEAD_PTS>& headPoints() const { return headPoints_; }

    void setCavPenaltyWeight(float w) { cavPenaltyWeight_ = w; }
    void setHeadPenaltyWeight(float w) { headPenaltyWeight_ = w; }

private:
    uint8_t unitId_;
    float   ratedPowerMW_;
    float   ratedHeadM_;
    float   cavPenaltyWeight_;
    float   headPenaltyWeight_;

    std::array<float, HILL_LOAD_PTS> loadPoints_;
    std::array<float, HILL_HEAD_PTS> headPoints_;
    std::array<std::array<float, HILL_LOAD_PTS>, HILL_HEAD_PTS> hillChart_;

    float bilinearInterpolate(float loadPu, float headPu) const;
    void initDefaultPoints();
};

struct MILPSolution {
    bool                                      feasible;
    bool                                      optimal;
    float                                     objective;
    float                                     mipGap;
    uint64_t                                  nodesExplored;
    uint64_t                                  iterations;
    std::array<std::array<float, SCHED_HOURS>, SCHED_UNITS> power;
    std::array<std::array<int,   SCHED_HOURS>, SCHED_UNITS> u;
    std::array<std::array<int,   SCHED_HOURS>, SCHED_UNITS> v;
    std::array<std::array<int,   SCHED_HOURS>, SCHED_UNITS> w;
    std::array<float, SCHED_HOURS>                            reserveSlack;
};

class MILPFormulation {
public:
    MILPFormulation();

    void setWeights(float wEff, float wCav, float wStart) {
        wEff_ = wEff; wCav_ = wCav; wStart_ = wStart;
    }

    void setMinStableHours(uint8_t h) { min_stable_hours_ = h; }

    void setUnitParams(uint8_t idx,
                       float pMin, float pMax,
                       float minUp, float minDown,
                       float rampRate,
                       const UnitEfficiencyCurve& curve);

    void setLoadCurve(const std::array<float, SCHED_HOURS>& load) { loadCurve_ = load; }
    void setReserveMarginPct(float pct) { reserveMarginPct_ = pct; }
    void setCavitationMax(float cav) { cavMax_ = cav; }
    void setInitialStates(const std::array<float, SCHED_UNITS>& lastPower,
                          const std::array<int,   SCHED_UNITS>& onOff,
                          const std::array<float, SCHED_UNITS>& operatingHours);

    void setHeadForecast(const std::array<float, SCHED_HOURS>& head) { headForecast_ = head; }

    void setCavitationCoeffs(const std::array<float, SCHED_UNITS>& cavPenaltyAdjust) {
        cavPenaltyAdjust_ = cavPenaltyAdjust;
    }

    void setGapTolerance(float gap) { gapTolerance_ = gap; }
    void setTimeLimitMs(uint64_t ms) { timeLimitMs_ = ms; }
    void setMaxNodes(uint64_t n) { maxNodes_ = n; }

    MILPSolution solve();

    struct UnitParams {
        float pMin;
        float pMax;
        float minUp;
        float minDown;
        float rampRate;
        UnitEfficiencyCurve curve;
    };

private:
    float wEff_;
    float wCav_;
    float wStart_;
    float reserveMarginPct_;
    float cavMax_;
    float gapTolerance_;
    uint64_t timeLimitMs_;
    uint64_t maxNodes_;
    uint8_t  min_stable_hours_;

    std::array<UnitParams, SCHED_UNITS> units_;
    std::array<float, SCHED_HOURS> loadCurve_;
    std::array<float, SCHED_HOURS> headForecast_;
    std::array<float, SCHED_UNITS> lastPower_;
    std::array<int,   SCHED_UNITS> initialOnOff_;
    std::array<float, SCHED_UNITS> initialOpHours_;
    std::array<float, SCHED_UNITS> cavPenaltyAdjust_;

    using Matrix = std::vector<std::vector<double>>;
    using Vector = std::vector<double>;

    struct LPModel {
        Matrix A;
        Vector b;
        Vector c;
        Vector lb;
        Vector ub;
        std::vector<int> isBinary;
        int numVars;
        int numCons;
    };

    struct LPSolution {
        bool   feasible;
        bool   optimal;
        double objective;
        Vector x;
        Vector duals;
    };

    struct BBNode {
        LPModel  model;
        double   lowerBound;
        std::vector<int> fixedVars;
        std::vector<int> fixedVals;
        int      depth;
    };

    void buildLP(LPModel& model);
    LPSolution solveSimplex(LPModel& model);
    bool phase1(Matrix& tableau, int nVars, int nSlack, int nArt, std::vector<int>& basis);
    bool phase2(Matrix& tableau, int nVars, int nSlack, std::vector<int>& basis, double& objVal);
    int  findEntering(const Vector& reduced, const std::vector<int>& basis);
    int  findLeaving(const Matrix& tab, int entering, int nRows);
    void pivot(Matrix& tab, int entering, int leaving, int nCols, int nRows,
               std::vector<int>& basis);
    void applyFixesToModel(LPModel& model,
                           const std::vector<int>& fixedVars,
                           const std::vector<int>& fixedVals);
    bool  isIntegerFeasible(const Vector& x, const std::vector<int>& isBinary);
    int   selectBranchingVar(const Vector& x, const std::vector<int>& isBinary,
                             const std::vector<int>& fixedVars);
    double evaluateObjective(const Vector& x);
    void   extractSolution(const Vector& x, MILPSolution& sol);
    void   applyInitialStateFixes(LPModel& model);
};

class PlantScheduler {
public:
    using ScheduleCallback = std::function<void(const PlantSchedule&)>;

    PlantScheduler();
    ~PlantScheduler();

    bool init(const Config& config);

    void setConfig(const SchedulerConfig& cfg) { cfg_ = cfg; }
    const SchedulerConfig& config() const { return cfg_; }

    PlantSchedule schedule(
        const std::array<float, SCHED_HOURS>& targetLoadCurve,
        const std::array<float, SCHED_UNITS>& currentPower,
        const std::array<int,   SCHED_UNITS>& currentOnOff,
        const std::array<float, SCHED_UNITS>& operatingHours);

    void asyncSchedule(
        const std::array<float, SCHED_HOURS>& targetLoadCurve,
        const std::array<float, SCHED_UNITS>& currentPower,
        const std::array<int,   SCHED_UNITS>& currentOnOff,
        const std::array<float, SCHED_UNITS>& operatingHours);

    void setScheduleCallback(ScheduleCallback cb) { cb_ = std::move(cb); }

    void updateCavitationStates(const std::vector<CavitationState>& cavitationMsg);

    PlantSchedule getSchedule() const { std::lock_guard<std::mutex> lk(mutex_); return lastSchedule_; }

    const MILPSolution& lastMIPSolution() const { std::lock_guard<std::mutex> lk(mutex_); return lastSol_; }

    float currentCavityPenalty(uint8_t unit) const {
        std::lock_guard<std::mutex> lk(mutex_);
        return unit < SCHED_UNITS ? cavPenaltyMultiplier_[unit] : 1.0f;
    }

    void setHeadForecast(const std::array<float, SCHED_HOURS>& head) {
        std::lock_guard<std::mutex> lk(mutex_);
        headForecast_ = head;
    }

    uint8_t minStableHours() const { return cfg_.min_stable_hours; }
    void setMinStableHours(uint8_t h) { cfg_.min_stable_hours = h; }

    void start();
    void stop();

    bool isRunning() const { return running_.load(); }
    uint64_t solveCount() const { return solve_count_.load(); }
    float lastSolveTimeMs() const { return last_solve_ms_.load(); }

    MILPFormulation& milp() { return milp_; }

private:
    void workerLoop();

    Config full_cfg_;
    SchedulerConfig cfg_;
    MILPFormulation milp_;
    std::array<UnitEfficiencyCurve, SCHED_UNITS> efficiencyCurves_;

    mutable std::mutex mutex_;
    PlantSchedule   lastSchedule_;
    MILPSolution    lastSol_;
    uint8_t         scheduleIdCounter_;

    std::array<float, SCHED_UNITS> cavPenaltyMultiplier_;
    std::array<uint64_t, SCHED_UNITS> lastCavityUpdateMs_;
    std::array<float, SCHED_HOURS> headForecast_;

    ScheduleCallback cb_;

    std::atomic<bool> running_{false};
    std::atomic<bool> task_pending_{false};
    std::atomic<uint64_t> solve_count_{0};
    std::atomic<float> last_solve_ms_{0.0f};

    std::unique_ptr<std::thread> worker_;
    std::mutex cv_mutex_;
    std::condition_variable cv_;

    std::array<float, SCHED_HOURS> pending_load_;
    std::array<float, SCHED_UNITS> pending_power_;
    std::array<int,   SCHED_UNITS> pending_onoff_;
    std::array<float, SCHED_UNITS> pending_op_hrs_;

    void initUnitCurves();
    void populatePlantSchedule(PlantSchedule& ps, const MILPSolution& sol,
                               const std::array<float, SCHED_HOURS>& load,
                               uint64_t now);
    float computeOptimizedEfficiency(const MILPSolution& sol);
    float computeCavitationReduction(const MILPSolution& sol,
                                     const std::array<float, SCHED_UNITS>& curPow);
};

}
