#include "../include/unit_scheduler.h"
#include <numeric>
#include <iostream>
#include <chrono>
#include <cfloat>
#include <climits>

namespace turbine_monitor {

// ============================================================
// UnitEfficiencyCurve Implementation
// ============================================================

UnitEfficiencyCurve::UnitEfficiencyCurve()
    : unitId_(0), ratedPowerMW_(150.0f), ratedHeadM_(80.0f),
      cavPenaltyWeight_(1.0f), headPenaltyWeight_(0.5f) {
    initDefaultPoints();
    for (auto& row : hillChart_) {
        row.fill(0.0f);
    }
}

void UnitEfficiencyCurve::initDefaultPoints() {
    for (int i = 0; i < HILL_LOAD_PTS; ++i) {
        loadPoints_[i] = 0.1f + 0.1f * i;
    }
    headPoints_[0] = 0.7f;
    headPoints_[1] = 0.85f;
    headPoints_[2] = 1.0f;
    headPoints_[3] = 1.1f;
    headPoints_[4] = 1.2f;
}

void UnitEfficiencyCurve::setHillChart(
    const std::array<std::array<float, HILL_LOAD_PTS>, HILL_HEAD_PTS>& table) {
    hillChart_ = table;
}

float UnitEfficiencyCurve::bilinearInterpolate(float loadPu, float headPu) const {
    int li = 0, hi = 0;
    for (int i = 0; i < HILL_LOAD_PTS - 1; ++i) {
        if (loadPu >= loadPoints_[i] && loadPu <= loadPoints_[i + 1]) {
            li = i; break;
        }
    }
    for (int i = 0; i < HILL_HEAD_PTS - 1; ++i) {
        if (headPu >= headPoints_[i] && headPu <= headPoints_[i + 1]) {
            hi = i; break;
        }
    }
    if (loadPu < loadPoints_.front()) li = 0;
    if (loadPu > loadPoints_.back())  li = HILL_LOAD_PTS - 2;
    if (headPu < headPoints_.front()) hi = 0;
    if (headPu > headPoints_.back())  hi = HILL_HEAD_PTS - 2;

    float x0 = loadPoints_[li], x1 = loadPoints_[li + 1];
    float y0 = headPoints_[hi], y1 = headPoints_[hi + 1];
    float dx = (x1 - x0) > 1e-6f ? (loadPu - x0) / (x1 - x0) : 0.0f;
    float dy = (y1 - y0) > 1e-6f ? (headPu - y0) / (y1 - y0) : 0.0f;

    float z00 = hillChart_[hi][li];
    float z10 = hillChart_[hi][li + 1];
    float z01 = hillChart_[hi + 1][li];
    float z11 = hillChart_[hi + 1][li + 1];

    return (1 - dx) * (1 - dy) * z00
         + dx       * (1 - dy) * z10
         + (1 - dx) * dy       * z01
         + dx       * dy       * z11;
}

float UnitEfficiencyCurve::efficiency(float powerMW, float headM) const {
    if (ratedPowerMW_ < 1e-6f || ratedHeadM_ < 1e-6f) return 0.0f;
    float loadPu = std::max(0.05f, std::min(1.05f, powerMW / ratedPowerMW_));
    float headPu = std::max(0.65f, std::min(1.25f, headM / ratedHeadM_));
    float eta = bilinearInterpolate(loadPu, headPu);
    return std::max(0.0f, std::min(0.99f, eta));
}

float UnitEfficiencyCurve::cavitationPenalty(float powerMW, float headM) const {
    if (ratedPowerMW_ < 1e-6f || ratedHeadM_ < 1e-6f) return 0.0f;
    float loadPu = powerMW / ratedPowerMW_;
    float headPu = headM / ratedHeadM_;

    float designCenter = 0.8f;
    float designWidth  = 0.1f;
    float loadPenalty = cavPenaltyWeight_ *
        std::pow(std::max(0.0f, std::abs(loadPu - designCenter) - designWidth), 2.0f);

    float headDeviation = std::abs(headPu - 1.0f);
    float headPenalty = headPenaltyWeight_ * headDeviation;

    if (loadPu < 0.2f) {
        loadPenalty += 2.0f * cavPenaltyWeight_ * std::pow(0.2f - loadPu, 2.0f);
    }
    if (loadPu > 1.0f) {
        loadPenalty += 1.5f * cavPenaltyWeight_ * std::pow(loadPu - 1.0f, 2.0f);
    }

    return loadPenalty + headPenalty;
}

float UnitEfficiencyCurve::startupCost(float operatingHours) const {
    float baseCost = ratedPowerMW_ * 0.8f;
    float warmDiscount = std::exp(-operatingHours / 4.0f);
    float thermalFactor = 0.3f + 0.7f * (1.0f - warmDiscount);
    return baseCost * thermalFactor;
}

// ============================================================
// MILPFormulation Implementation
// ============================================================

MILPFormulation::MILPFormulation()
    : wEff_(2.0f), wCav_(5.0f), wStart_(1.0f),
      reserveMarginPct_(0.05f), cavMax_(0.5f),
      gapTolerance_(0.001f), timeLimitMs_(5000), maxNodes_(10000) {
    for (auto& f : loadCurve_) f = 0.0f;
    for (auto& f : headForecast_) f = 80.0f;
    for (auto& f : lastPower_) f = 0.0f;
    for (auto& i : initialOnOff_) i = 0;
    for (auto& f : initialOpHours_) f = 100.0f;
    for (auto& f : cavPenaltyAdjust_) f = 1.0f;
}

void MILPFormulation::setUnitParams(uint8_t idx,
                                    float pMin, float pMax,
                                    float minUp, float minDown,
                                    float rampRate,
                                    const UnitEfficiencyCurve& curve) {
    if (idx >= SCHED_UNITS) return;
    units_[idx].pMin = pMin;
    units_[idx].pMax = pMax;
    units_[idx].minUp = minUp;
    units_[idx].minDown = minDown;
    units_[idx].rampRate = rampRate;
    units_[idx].curve = curve;
}

// Variable layout for 6 units x 24 hours:
// P[i,t]   : 0..143  (i*24 + t)
// u[i,t]   : 144..287 (144 + i*24 + t)
// v[i,t]   : 288..431 (288 + i*24 + t)
// w[i,t]   : 432..575 (432 + i*24 + t)
// s_res[t] : 576..599 (slack for reserve, >=0)
// cav_aux[i,t]: 600..743 (aux for cavitation risk linearization)
// Total: 744 variables

static inline int var_P(int i, int t) { return i * SCHED_HOURS + t; }
static inline int var_u(int i, int t) { return 144 + i * SCHED_HOURS + t; }
static inline int var_v(int i, int t) { return 288 + i * SCHED_HOURS + t; }
static inline int var_w(int i, int t) { return 432 + i * SCHED_HOURS + t; }
static inline int var_sres(int t)    { return 576 + t; }
static inline int var_cav(int i, int t){ return 600 + i * SCHED_HOURS + t; }
static constexpr int TOTAL_VARS = 600 + SCHED_UNITS * SCHED_HOURS;

void MILPFormulation::buildLP(LPModel& model) {
    model.numVars = TOTAL_VARS;
    model.numCons = 0;
    model.A.clear();
    model.b.clear();
    model.c.assign(model.numVars, 0.0);
    model.lb.assign(model.numVars, 0.0);
    model.ub.assign(model.numVars, 0.0);
    model.isBinary.assign(model.numVars, 0);

    for (int i = 0; i < SCHED_UNITS; ++i) {
        float pMin = units_[i].pMin;
        float pMax = units_[i].pMax;
        for (int t = 0; t < SCHED_HOURS; ++t) {
            int pIdx = var_P(i, t);
            model.lb[pIdx] = 0.0;
            model.ub[pIdx] = pMax;

            int uIdx = var_u(i, t);
            model.lb[uIdx] = 0.0; model.ub[uIdx] = 1.0;
            model.isBinary[uIdx] = 1;

            int vIdx = var_v(i, t);
            model.lb[vIdx] = 0.0; model.ub[vIdx] = 1.0;
            model.isBinary[vIdx] = 1;

            int wIdx = var_w(i, t);
            model.lb[wIdx] = 0.0; model.ub[wIdx] = 1.0;
            model.isBinary[wIdx] = 1;

            int cavIdx = var_cav(i, t);
            model.lb[cavIdx] = 0.0;
            model.ub[cavIdx] = cavMax_ * 2.0f;
            model.isBinary[cavIdx] = 0;

            double startCost = units_[i].curve.startupCost(initialOpHours_[i]);
            model.c[vIdx] = wStart_ * startCost;

            model.c[cavIdx] = wCav_ * cavPenaltyAdjust_[i];
        }
    }

    for (int t = 0; t < SCHED_HOURS; ++t) {
        int sIdx = var_sres(t);
        model.lb[sIdx] = 0.0;
        model.ub[sIdx] = 1e9;
        model.c[sIdx] = 0.0;
    }

    for (int i = 0; i < SCHED_UNITS; ++i) {
        for (int t = 0; t < SCHED_HOURS; ++t) {
            int pIdx = var_P(i, t);
            int uIdx = var_u(i, t);
            {
                std::vector<double> row(model.numVars, 0.0);
                row[pIdx] = 1.0;
                row[uIdx] = -units_[i].pMax;
                model.A.push_back(std::move(row));
                model.b.push_back(0.0);
                model.numCons++;
            }
            {
                std::vector<double> row(model.numVars, 0.0);
                row[pIdx] = -1.0;
                row[uIdx] = units_[i].pMin;
                model.A.push_back(std::move(row));
                model.b.push_back(0.0);
                model.numCons++;
            }
        }
    }

    for (int t = 0; t < SCHED_HOURS; ++t) {
        std::vector<double> row(model.numVars, 0.0);
        float reserve = loadCurve_[t] * reserveMarginPct_;
        for (int i = 0; i < SCHED_UNITS; ++i) {
            row[var_P(i, t)] = 1.0;
        }
        row[var_sres(t)] = 1.0;
        model.A.push_back(std::move(row));
        model.b.push_back(static_cast<double>(loadCurve_[t] + reserve));
        model.numCons++;

        std::vector<double> row2(model.numVars, 0.0);
        for (int i = 0; i < SCHED_UNITS; ++i) {
            row2[var_P(i, t)] = -1.0;
        }
        row2[var_sres(t)] = -1.0;
        model.A.push_back(std::move(row2));
        model.b.push_back(static_cast<double>(-(loadCurve_[t] + reserve)));
        model.numCons++;
    }

    for (int i = 0; i < SCHED_UNITS; ++i) {
        for (int t = 0; t < SCHED_HOURS; ++t) {
            int uCur  = var_u(i, t);
            int uPrev = (t == 0) ? -1 : var_u(i, t - 1);
            int vCur  = var_v(i, t);
            int wCur  = var_w(i, t);

            std::vector<double> row(model.numVars, 0.0);
            row[uCur] = 1.0;
            if (uPrev >= 0) {
                row[uPrev] = -1.0;
            } else {
                row[uCur] += 1.0;
                row[vCur] = -1.0;
                row[wCur] = 1.0;
                model.A.push_back(std::move(row));
                model.b.push_back(static_cast<double>(initialOnOff_[i]));
                model.numCons++;
                continue;
            }
            row[vCur] = -1.0;
            row[wCur] = 1.0;
            model.A.push_back(std::move(row));
            model.b.push_back(0.0);
            model.numCons++;
        }
    }

    for (int i = 0; i < SCHED_UNITS; ++i) {
        for (int t = 0; t < SCHED_HOURS; ++t) {
            int vCur = var_v(i, t);
            int wCur = var_w(i, t);
            std::vector<double> row(model.numVars, 0.0);
            row[vCur] = 1.0;
            row[wCur] = 1.0;
            model.A.push_back(std::move(row));
            model.b.push_back(1.0);
            model.numCons++;
        }
    }

    for (int i = 0; i < SCHED_UNITS; ++i) {
        int minUpSteps = static_cast<int>(std::ceil(units_[i].minUp / 3600.0f));
        minUpSteps = std::min(minUpSteps, SCHED_HOURS);
        int minDownSteps = static_cast<int>(std::ceil(units_[i].minDown / 3600.0f));
        minDownSteps = std::min(minDownSteps, SCHED_HOURS);

        for (int t = 0; t < SCHED_HOURS; ++t) {
            int vIdx = var_v(i, t);
            std::vector<double> row(model.numVars, 0.0);
            row[vIdx] = -1.0;
            int cnt = 0;
            for (int k = 0; k < minUpSteps && t + k < SCHED_HOURS; ++k) {
                row[var_u(i, t + k)] = 1.0;
                cnt++;
            }
            if (cnt > 0) {
                model.A.push_back(std::move(row));
                model.b.push_back(0.0);
                model.numCons++;
            }
        }

        for (int t = 0; t < SCHED_HOURS; ++t) {
            int wIdx = var_w(i, t);
            std::vector<double> row(model.numVars, 0.0);
            row[wIdx] = -1.0;
            int cnt = 0;
            for (int k = 0; k < minDownSteps && t + k < SCHED_HOURS; ++k) {
                row[var_u(i, t + k)] = -1.0;
                cnt++;
            }
            if (cnt > 0) {
                model.A.push_back(std::move(row));
                model.b.push_back(-static_cast<double>(cnt));
                model.numCons++;
            }
        }
    }

    for (int i = 0; i < SCHED_UNITS; ++i) {
        for (int t = 0; t < SCHED_HOURS; ++t) {
            int pCur = var_P(i, t);
            int pPrev = (t == 0) ? -1 : var_P(i, t - 1);
            double ramp = units_[i].rampRate;

            if (t == 0) {
                std::vector<double> row(model.numVars, 0.0);
                row[pCur] = 1.0;
                model.A.push_back(std::move(row));
                model.b.push_back(static_cast<double>(lastPower_[i] + ramp));
                model.numCons++;

                std::vector<double> row2(model.numVars, 0.0);
                row2[pCur] = -1.0;
                model.A.push_back(std::move(row2));
                model.b.push_back(static_cast<double>(-(lastPower_[i] - ramp)));
                model.numCons++;
            } else {
                std::vector<double> row(model.numVars, 0.0);
                row[pCur] = 1.0;
                row[pPrev] = -1.0;
                model.A.push_back(std::move(row));
                model.b.push_back(ramp);
                model.numCons++;

                std::vector<double> row2(model.numVars, 0.0);
                row2[pCur] = -1.0;
                row2[pPrev] = 1.0;
                model.A.push_back(std::move(row2));
                model.b.push_back(ramp);
                model.numCons++;
            }
        }
    }

    for (int i = 0; i < SCHED_UNITS; ++i) {
        for (int t = 0; t < SCHED_HOURS; ++t) {
            int cavIdx = var_cav(i, t);
            int pIdx = var_P(i, t);
            float pMax = units_[i].pMax;
            float hT = headForecast_[t];
            float pMid = units_[i].curve.ratedPowerMW() * 0.8f;
            float coeff = 4.0f * cavPenaltyAdjust_[i] / (pMax * pMax);

            {
                std::vector<double> row(model.numVars, 0.0);
                row[cavIdx] = -1.0;
                row[pIdx]   = coeff * pMax;
                model.A.push_back(std::move(row));
                model.b.push_back(static_cast<double>(coeff * pMax * (pMax - pMid)));
                model.numCons++;
            }
            {
                std::vector<double> row(model.numVars, 0.0);
                row[cavIdx] = -1.0;
                row[pIdx]   = -coeff * pMax;
                model.A.push_back(std::move(row));
                model.b.push_back(static_cast<double>(coeff * pMax * pMid));
                model.numCons++;
            }
            {
                std::vector<double> row(model.numVars, 0.0);
                row[cavIdx] = 1.0;
                model.A.push_back(std::move(row));
                model.b.push_back(static_cast<double>(cavMax_));
                model.numCons++;
            }

            {
                std::vector<double> row(model.numVars, 0.0);
                row[pIdx]   = wEff_ * 2.0 / (pMax * pMax);
                row[cavIdx] = wCav_ * cavPenaltyAdjust_[i];
                (void)hT;
            }
        }
    }

    for (int i = 0; i < SCHED_UNITS; ++i) {
        for (int t = 0; t < SCHED_HOURS; ++t) {
            int pIdx = var_P(i, t);
            float pMax = units_[i].pMax;
            model.c[pIdx] += wEff_ * 0.01 / pMax;
        }
    }
}

void MILPFormulation::applyInitialStateFixes(LPModel& model) {
    for (int i = 0; i < SCHED_UNITS; ++i) {
        if (initialOnOff_[i] == 1) {
            float minUpLeft = units_[i].minUp;
            int forcedSteps = static_cast<int>(std::ceil(minUpLeft / 3600.0f));
            forcedSteps = std::min(forcedSteps, 2);
            for (int t = 0; t < forcedSteps && t < SCHED_HOURS; ++t) {
                model.lb[var_u(i, t)] = 1.0;
                model.ub[var_u(i, t)] = 1.0;
                model.lb[var_v(i, t)] = 0.0;
                model.ub[var_v(i, t)] = 0.0;
            }
        } else {
            float minDownLeft = units_[i].minDown;
            int forcedSteps = static_cast<int>(std::ceil(minDownLeft / 3600.0f));
            forcedSteps = std::min(forcedSteps, 2);
            for (int t = 0; t < forcedSteps && t < SCHED_HOURS; ++t) {
                model.lb[var_u(i, t)] = 0.0;
                model.ub[var_u(i, t)] = 0.0;
                model.lb[var_w(i, t)] = 0.0;
                model.ub[var_w(i, t)] = 0.0;
            }
        }
    }
}

void MILPFormulation::applyFixesToModel(LPModel& model,
                                        const std::vector<int>& fixedVars,
                                        const std::vector<int>& fixedVals) {
    for (size_t k = 0; k < fixedVars.size(); ++k) {
        int vid = fixedVars[k];
        int val = fixedVals[k];
        model.lb[vid] = val;
        model.ub[vid] = val;
    }
}

int MILPFormulation::findEntering(const Vector& reduced, const std::vector<int>& basis) {
    int n = static_cast<int>(reduced.size());
    std::vector<int> inBasis(n, 0);
    for (int b : basis) inBasis[b] = 1;

    int best = -1;
    double bestVal = 0.0;
    for (int j = 0; j < n; ++j) {
        if (inBasis[j]) continue;
        if (reduced[j] < bestVal - 1e-9) {
            bestVal = reduced[j];
            best = j;
        }
    }
    return best;
}

int MILPFormulation::findLeaving(const Matrix& tab, int entering, int nRows) {
    int nCols = static_cast<int>(tab[0].size());
    int rhsCol = nCols - 1;

    int best = -1;
    double bestRatio = 1e100;
    for (int i = 0; i < nRows; ++i) {
        double coeff = tab[i][entering];
        if (coeff <= 1e-9) continue;
        double ratio = tab[i][rhsCol] / coeff;
        if (ratio < bestRatio - 1e-12 ||
            (std::fabs(ratio - bestRatio) < 1e-9 && i < best)) {
            bestRatio = ratio;
            best = i;
        }
    }
    return best;
}

void MILPFormulation::pivot(Matrix& tab, int entering, int leaving, int nCols, int nRows,
                            std::vector<int>& basis) {
    double pivotVal = tab[leaving][entering];
    if (std::fabs(pivotVal) < 1e-15) return;

    for (int j = 0; j < nCols; ++j) {
        tab[leaving][j] /= pivotVal;
    }

    for (int i = 0; i < nRows; ++i) {
        if (i == leaving) continue;
        double factor = tab[i][entering];
        if (std::fabs(factor) < 1e-15) continue;
        for (int j = 0; j < nCols; ++j) {
            tab[i][j] -= factor * tab[leaving][j];
        }
    }

    basis[leaving] = entering;
}

bool MILPFormulation::phase1(Matrix& tableau, int nVars, int nSlack, int nArt,
                             std::vector<int>& basis) {
    int nRows = static_cast<int>(tableau.size());
    int nCols = static_cast<int>(tableau[0].size());

    for (int i = 0; i < nRows; ++i) {
        if (tableau[i][nCols - 1] < -1e-9) {
            for (int j = 0; j < nCols; ++j) {
                tableau[i][j] = -tableau[i][j];
            }
        }
    }

    {
        std::vector<double> wRow(nCols, 0.0);
        for (int i = 0; i < nRows; ++i) {
            int artCol = nVars + nSlack + i;
            if (artCol < nCols - 1) {
                wRow[artCol] += 1.0;
                for (int j = 0; j < nCols; ++j) {
                    wRow[j] -= tableau[i][j];
                }
            }
        }
        tableau.push_back(std::move(wRow));
    }
    int wRowIdx = static_cast<int>(tableau.size()) - 1;
    int maxIter = nRows * nCols * 10;

    for (int iter = 0; iter < maxIter; ++iter) {
        int entering = -1;
        double bestRed = 0.0;
        for (int j = 0; j < nCols - 1; ++j) {
            if (tableau[wRowIdx][j] < bestRed - 1e-8) {
                bestRed = tableau[wRowIdx][j];
                entering = j;
            }
        }
        if (entering < 0) break;

        int leaving = findLeaving(tableau, entering, nRows);
        if (leaving < 0) {
            return false;
        }
        pivot(tableau, entering, leaving, nCols, nRows, basis);

        double pivotVal = tableau[leaving][entering];
        double factor = tableau[wRowIdx][entering];
        if (std::fabs(factor) > 1e-15) {
            for (int j = 0; j < nCols; ++j) {
                tableau[wRowIdx][j] -= factor / pivotVal * tableau[leaving][j];
            }
        }
    }

    if (tableau[wRowIdx][nCols - 1] > 1e-6) {
        tableau.pop_back();
        return false;
    }
    tableau.pop_back();
    return true;
}

bool MILPFormulation::phase2(Matrix& tableau, int nVars, int nSlack,
                             std::vector<int>& basis, double& objVal) {
    int nRows = static_cast<int>(tableau.size());
    int nCols = static_cast<int>(tableau[0].size());
    int objRowIdx = nRows;

    {
        std::vector<double> objRow(nCols, 0.0);
        for (int j = 0; j < nVars; ++j) {
            // obj coefficients will be reconstructed later
        }
        tableau.push_back(std::move(objRow));
    }

    int maxIter = nVars * nRows * 20;
    for (int iter = 0; iter < maxIter; ++iter) {
        int entering = -1;
        double bestRed = 0.0;
        for (int j = 0; j < nCols - 1; ++j) {
            if (tableau[objRowIdx][j] < bestRed - 1e-8) {
                bestRed = tableau[objRowIdx][j];
                entering = j;
            }
        }
        if (entering < 0) {
            objVal = -tableau[objRowIdx][nCols - 1];
            tableau.pop_back();
            return true;
        }

        int leaving = findLeaving(tableau, entering, nRows);
        if (leaving < 0) {
            tableau.pop_back();
            return false;
        }
        pivot(tableau, entering, leaving, nCols, nRows, basis);

        double factor = tableau[objRowIdx][entering];
        double pivVal = tableau[leaving][entering];
        if (std::fabs(factor) > 1e-15) {
            for (int j = 0; j < nCols; ++j) {
                tableau[objRowIdx][j] -= factor / pivVal * tableau[leaving][j];
            }
        }
    }

    objVal = -tableau[objRowIdx][nCols - 1];
    tableau.pop_back();
    return true;
}

MILPFormulation::LPSolution MILPFormulation::solveSimplex(LPModel& model) {
    LPSolution sol;
    sol.feasible = false;
    sol.optimal = false;
    sol.objective = 1e100;
    sol.x.assign(model.numVars, 0.0);

    int nVars = model.numVars;
    int nCons = model.numCons;
    if (nCons == 0 || nVars == 0) return sol;

    int nSlack = nCons;
    int nArt = nCons;
    int totalCols = nVars + nSlack + nArt + 1;

    Matrix tableau;
    tableau.reserve(nCons);

    for (int i = 0; i < nCons; ++i) {
        std::vector<double> row(totalCols, 0.0);
        for (int j = 0; j < nVars; ++j) {
            row[j] = model.A[i][j];
        }
        int slackCol = nVars + i;
        row[slackCol] = 1.0;
        int artCol = nVars + nSlack + i;
        row[artCol] = 1.0;
        row[totalCols - 1] = model.b[i];
        if (row[totalCols - 1] < -1e-9) {
            for (int j = 0; j < totalCols; ++j) row[j] = -row[j];
        }
        tableau.push_back(std::move(row));
    }

    std::vector<int> basis(nCons);
    for (int i = 0; i < nCons; ++i) {
        basis[i] = nVars + nSlack + i;
    }

    if (!phase1(tableau, nVars, nSlack, nArt, basis)) {
        return sol;
    }
    sol.feasible = true;

    for (auto& row : tableau) {
        row.erase(row.begin() + nVars + nSlack, row.begin() + nVars + nSlack + nArt);
    }
    int phase2Cols = nVars + nSlack + 1;

    int objRow = static_cast<int>(tableau.size());
    std::vector<double> objRowVec(phase2Cols, 0.0);
    for (int j = 0; j < nVars; ++j) {
        objRowVec[j] = model.c[j];
    }
    tableau.push_back(std::move(objRowVec));

    for (int i = 0; i < nCons; ++i) {
        int bvar = basis[i];
        if (bvar < nVars) {
            double coef = tableau[objRow][bvar];
            if (std::fabs(coef) > 1e-12) {
                for (int j = 0; j < phase2Cols; ++j) {
                    tableau[objRow][j] -= coef * tableau[i][j];
                }
            }
        }
    }

    int maxIter = nVars * nCons * 20;
    for (int iter = 0; iter < maxIter; ++iter) {
        int entering = -1;
        double bestRed = 0.0;
        for (int j = 0; j < phase2Cols - 1; ++j) {
            if (tableau[objRow][j] < bestRed - 1e-8) {
                bestRed = tableau[objRow][j];
                entering = j;
            }
        }
        if (entering < 0) {
            sol.optimal = true;
            sol.objective = -tableau[objRow][phase2Cols - 1];
            break;
        }

        int leaving = findLeaving(tableau, entering, nCons);
        if (leaving < 0) {
            sol.optimal = false;
            sol.objective = -1e100;
            tableau.pop_back();
            return sol;
        }

        pivot(tableau, entering, leaving, phase2Cols, nCons, basis);
        double factor = tableau[objRow][entering];
        double pv = tableau[leaving][entering];
        if (std::fabs(factor) > 1e-15 && std::fabs(pv) > 1e-15) {
            for (int j = 0; j < phase2Cols; ++j) {
                tableau[objRow][j] -= factor / pv * tableau[leaving][j];
            }
        }
    }

    for (int i = 0; i < nCons; ++i) {
        int bvar = basis[i];
        if (bvar < nVars) {
            sol.x[bvar] = tableau[i][phase2Cols - 1];
        }
    }

    for (int j = 0; j < nVars; ++j) {
        sol.x[j] = std::max(model.lb[j], std::min(model.ub[j], sol.x[j]));
    }

    tableau.pop_back();
    return sol;
}

bool MILPFormulation::isIntegerFeasible(const Vector& x, const std::vector<int>& isBinary) {
    for (size_t j = 0; j < isBinary.size(); ++j) {
        if (isBinary[j]) {
            double v = x[j];
            if (v > 0.001 && v < 0.999) return false;
        }
    }
    return true;
}

int MILPFormulation::selectBranchingVar(const Vector& x, const std::vector<int>& isBinary,
                                        const std::vector<int>& fixedVars) {
    std::vector<int> isFixed(isBinary.size(), 0);
    for (int f : fixedVars) isFixed[f] = 1;

    int best = -1;
    double bestScore = -1.0;
    for (size_t j = 0; j < isBinary.size(); ++j) {
        if (!isBinary[j] || isFixed[j]) continue;
        double v = x[j];
        if (v <= 0.001 || v >= 0.999) continue;
        double dist = std::fabs(v - 0.5);
        double score = 1.0 - dist;
        if (score > bestScore) {
            bestScore = score;
            best = static_cast<int>(j);
        }
    }
    return best;
}

double MILPFormulation::evaluateObjective(const Vector& x) {
    double obj = 0.0;
    for (int i = 0; i < SCHED_UNITS; ++i) {
        for (int t = 0; t < SCHED_HOURS; ++t) {
            double p = x[var_P(i, t)];
            double u = x[var_u(i, t)];
            double v = x[var_v(i, t)];

            float hT = headForecast_[t];
            float eta = units_[i].curve.efficiency(static_cast<float>(p), hT);
            float cav = units_[i].curve.cavitationPenalty(static_cast<float>(p), hT);
            double startC = units_[i].curve.startupCost(initialOpHours_[i]);

            obj += wEff_ * (1.0 - eta) * u;
            obj += wCav_ * cav * cavPenaltyAdjust_[i] * u;
            obj += wStart_ * startC * v;
        }
    }
    return obj;
}

void MILPFormulation::extractSolution(const Vector& x, MILPSolution& sol) {
    for (int i = 0; i < SCHED_UNITS; ++i) {
        for (int t = 0; t < SCHED_HOURS; ++t) {
            sol.power[i][t] = static_cast<float>(x[var_P(i, t)]);
            double uv = x[var_u(i, t)];
            double vv = x[var_v(i, t)];
            double wv = x[var_w(i, t)];
            sol.u[i][t] = (uv > 0.5) ? 1 : 0;
            sol.v[i][t] = (vv > 0.5) ? 1 : 0;
            sol.w[i][t] = (wv > 0.5) ? 1 : 0;
        }
    }
    for (int t = 0; t < SCHED_HOURS; ++t) {
        sol.reserveSlack[t] = static_cast<float>(x[var_sres(t)]);
    }
}

MILPSolution MILPFormulation::solve() {
    auto t0 = std::chrono::steady_clock::now();

    MILPSolution sol;
    sol.feasible = false;
    sol.optimal = false;
    sol.objective = 1e10f;
    sol.mipGap = 1.0f;
    sol.nodesExplored = 0;
    sol.iterations = 0;

    for (auto& a : sol.power) a.fill(0.0f);
    for (auto& a : sol.u) a.fill(0);
    for (auto& a : sol.v) a.fill(0);
    for (auto& a : sol.w) a.fill(0);
    sol.reserveSlack.fill(0.0f);

    LPModel rootModel;
    buildLP(rootModel);
    applyInitialStateFixes(rootModel);

    LPSolution rootLP = solveSimplex(rootModel);
    if (!rootLP.feasible) {
        return sol;
    }

    double lowerBound = rootLP.objective;
    double upperBound = 1e100;
    Vector bestX = rootLP.x;

    if (isIntegerFeasible(rootLP.x, rootModel.isBinary)) {
        upperBound = evaluateObjective(rootLP.x);
        bestX = rootLP.x;
        sol.nodesExplored = 1;
        sol.iterations = 1;
        sol.feasible = true;
        sol.optimal = true;
        sol.objective = static_cast<float>(upperBound);
        sol.mipGap = upperBound > 1e-6f
            ? static_cast<float>((upperBound - lowerBound) / (std::fabs(upperBound) + 1e-9))
            : 0.0f;
        extractSolution(bestX, sol);
        return sol;
    }

    std::deque<BBNode> stack;
    BBNode root;
    root.model = rootModel;
    root.lowerBound = lowerBound;
    root.depth = 0;
    stack.push_back(std::move(root));

    uint64_t nodesExplored = 0;

    while (!stack.empty()) {
        auto tn = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(tn - t0).count();
        if (elapsed > static_cast<int64_t>(timeLimitMs_)) break;
        if (nodesExplored >= maxNodes_) break;

        BBNode node = std::move(stack.back());
        stack.pop_back();
        nodesExplored++;

        if (upperBound < 1e99 && node.lowerBound >= upperBound * (1.0 - 1e-9)) {
            continue;
        }

        applyFixesToModel(node.model, node.fixedVars, node.fixedVals);
        LPSolution lpSol = solveSimplex(node.model);
        if (!lpSol.feasible) continue;

        if (upperBound < 1e99 && lpSol.objective >= upperBound * (1.0 - 1e-9)) {
            continue;
        }

        if (isIntegerFeasible(lpSol.x, node.model.isBinary)) {
            double realObj = evaluateObjective(lpSol.x);
            if (realObj < upperBound) {
                upperBound = realObj;
                bestX = lpSol.x;
            }
            double gap = upperBound > 1e-9
                ? (upperBound - lpSol.objective) / (std::fabs(upperBound) + 1e-9)
                : 0.0;
            if (gap < gapTolerance_) {
                break;
            }
            continue;
        }

        int branchVar = selectBranchingVar(lpSol.x, node.model.isBinary, node.fixedVars);
        if (branchVar < 0) continue;

        BBNode leftNode;
        leftNode.model = node.model;
        leftNode.fixedVars = node.fixedVars;
        leftNode.fixedVals = node.fixedVals;
        leftNode.fixedVars.push_back(branchVar);
        leftNode.fixedVals.push_back(1);
        leftNode.depth = node.depth + 1;

        double xVar = lpSol.x[branchVar];
        if (xVar > 0.5) {
            BBNode rightNode;
            rightNode.model = node.model;
            rightNode.fixedVars = node.fixedVars;
            rightNode.fixedVals = node.fixedVals;
            rightNode.fixedVars.push_back(branchVar);
            rightNode.fixedVals.push_back(0);
            rightNode.depth = node.depth + 1;
            stack.push_back(std::move(rightNode));
            stack.push_back(std::move(leftNode));
        } else {
            stack.push_back(std::move(leftNode));
            BBNode rightNode;
            rightNode.model = node.model;
            rightNode.fixedVars = node.fixedVars;
            rightNode.fixedVals = node.fixedVals;
            rightNode.fixedVars.push_back(branchVar);
            rightNode.fixedVals.push_back(0);
            rightNode.depth = node.depth + 1;
            stack.push_back(std::move(rightNode));
        }
    }

    sol.nodesExplored = nodesExplored;
    sol.iterations = nodesExplored;

    if (upperBound < 1e99) {
        sol.feasible = true;
        sol.objective = static_cast<float>(upperBound);
        double realLb = std::min(lowerBound, upperBound);
        sol.mipGap = upperBound > 1e-9f
            ? static_cast<float>((upperBound - realLb) / (std::fabs(upperBound) + 1e-9))
            : 0.0f;
        if (sol.mipGap < gapTolerance_) sol.optimal = true;
        extractSolution(bestX, sol);
    } else {
        sol.feasible = true;
        sol.optimal = false;
        sol.objective = static_cast<float>(rootLP.objective);
        extractSolution(rootLP.x, sol);
    }

    return sol;
}

// ============================================================
// PlantScheduler Implementation
// ============================================================

PlantScheduler::PlantScheduler()
    : scheduleIdCounter_(0) {
    for (auto& f : cavPenaltyMultiplier_) f = 1.0f;
    for (auto& u : lastCavityUpdateMs_) u = 0;
    for (auto& f : headForecast_) f = 80.0f;
}

bool PlantScheduler::init(const Config& config) {
    cfg_ = config;
    initUnitCurves();
    return true;
}

void PlantScheduler::initUnitCurves() {
    std::array<std::array<std::array<float, HILL_LOAD_PTS>, HILL_HEAD_PTS>, SCHED_UNITS> hillData;

    for (int h = 0; h < HILL_HEAD_PTS; ++h) {
        for (int l = 0; l < HILL_LOAD_PTS; ++l) {
            float loadPu = 0.1f + 0.1f * l;
            float headPu = 0.7f + 0.125f * h;
            float loadFactor = std::exp(-std::pow((loadPu - 0.8f) / 0.35f, 2.0f));
            float headFactor = std::exp(-std::pow((headPu - 1.0f) / 0.25f, 2.0f));
            float base = 0.70f + 0.22f * loadFactor * headFactor;
            for (int i = 0; i < SCHED_UNITS; ++i) {
                float var = 0.01f * std::sin(i * 1.7f + h * 0.5f + l * 0.3f);
                hillData[i][h][l] = std::max(0.55f, std::min(0.97f, base + var));
            }
        }
    }

    for (int i = 0; i < SCHED_UNITS; ++i) {
        efficiencyCurves_[i].setUnitId(static_cast<uint8_t>(i + 1));
        efficiencyCurves_[i].setRatedPowerMW(150.0f - i * 2.0f);
        efficiencyCurves_[i].setRatedHeadM(80.0f + i * 0.5f);
        efficiencyCurves_[i].setCavPenaltyWeight(1.0f + i * 0.1f);
        efficiencyCurves_[i].setHillChart(hillData[i]);
    }
}

PlantSchedule PlantScheduler::schedule(
    const std::array<float, SCHED_HOURS>& targetLoadCurve,
    const std::array<float, SCHED_UNITS>& currentPower,
    const std::array<int,   SCHED_UNITS>& currentOnOff,
    const std::array<float, SCHED_UNITS>& operatingHours) {

    std::lock_guard<std::mutex> lk(mutex_);
    uint64_t now = currentTimestampMs();

    milp_.setWeights(cfg_.scheduler.weight_efficiency,
                     cfg_.scheduler.weight_cavitation_penalty,
                     cfg_.scheduler.weight_startup_cost);
    milp_.setReserveMarginPct(cfg_.scheduler.reserve_margin_pct);
    milp_.setCavitationMax(cfg_.scheduler.max_cavitation_allow);
    milp_.setGapTolerance(cfg_.scheduler.mip_gap_tolerance);
    milp_.setTimeLimitMs(cfg_.scheduler.mip_time_limit_ms);
    milp_.setMaxNodes(5000);

    for (int i = 0; i < SCHED_UNITS; ++i) {
        milp_.setUnitParams(
            static_cast<uint8_t>(i),
            30.0f,
            efficiencyCurves_[i].ratedPowerMW(),
            cfg_.scheduler.min_up_time_s,
            cfg_.scheduler.min_down_time_s,
            cfg_.scheduler.ramp_rate_limit_mwps * 3600.0f,
            efficiencyCurves_[i]
        );
    }

    milp_.setLoadCurve(targetLoadCurve);
    milp_.setHeadForecast(headForecast_);
    milp_.setInitialStates(currentPower, currentOnOff, operatingHours);
    milp_.setCavitationCoeffs(cavPenaltyMultiplier_);

    MILPSolution mipSol = milp_.solve();
    lastSol_ = mipSol;

    PlantSchedule ps;
    populatePlantSchedule(ps, mipSol, targetLoadCurve, now);
    lastSchedule_ = ps;
    scheduleIdCounter_++;
    return ps;
}

void PlantScheduler::populatePlantSchedule(PlantSchedule& ps, const MILPSolution& sol,
                                           const std::array<float, SCHED_HOURS>& load,
                                           uint64_t now) {
    ps.timestamp = now;
    ps.status = sol.feasible ? (sol.optimal ? ScheduleStatus::CONVERGED : ScheduleStatus::PARTIAL_FEAS)
                             : ScheduleStatus::INFEASIBLE;
    ps.schedule_id = scheduleIdCounter_;
    ps.horizon_s = cfg_.scheduler.horizon_steps * cfg_.scheduler.time_step_s;

    float totalLoad = 0.0f;
    for (float l : load) totalLoad += l;
    ps.target_total_power_mw = totalLoad / SCHED_HOURS;

    float totalPower = 0.0f;
    ps.units.clear();
    for (int i = 0; i < SCHED_UNITS; ++i) {
        UnitSchedule us{};
        us.turbine_id = static_cast<uint8_t>(i + 1);
        float avgPow = 0.0f, avgEff = 0.0f, avgCav = 0.0f;
        int activeHours = 0;
        float stCost = 0.0f, sdCost = 0.0f;
        for (int t = 0; t < SCHED_HOURS; ++t) {
            avgPow += sol.power[i][t];
            if (sol.u[i][t]) {
                float eff = efficiencyCurves_[i].efficiency(sol.power[i][t], headForecast_[t]);
                float cav = efficiencyCurves_[i].cavitationPenalty(sol.power[i][t], headForecast_[t]);
                avgEff += eff;
                avgCav += cav;
                activeHours++;
            }
            if (sol.v[i][t]) stCost += efficiencyCurves_[i].startupCost(100.0f);
            if (sol.w[i][t]) sdCost += efficiencyCurves_[i].ratedPowerMW() * 0.3f;
        }
        avgPow /= SCHED_HOURS;
        avgEff = activeHours > 0 ? avgEff / activeHours : 0.0f;
        avgCav = activeHours > 0 ? avgCav / activeHours : 0.0f;
        us.is_active = avgPow > 1.0f;
        us.power_mw = avgPow;
        us.efficiency_pct = avgEff * 100.0f;
        us.cavitation_risk = avgCav;
        us.operating_hours = static_cast<float>(activeHours);
        us.startup_cost = stCost;
        us.shutdown_cost = sdCost;
        ps.units.push_back(us);
        totalPower += avgPow;
    }
    ps.current_total_power_mw = totalPower;
    ps.optimized_efficiency_pct = computeOptimizedEfficiency(sol);
    std::array<float, SCHED_UNITS> curPow{};
    ps.cavitation_risk_reduction_pct = computeCavitationReduction(sol, curPow);
    ps.mip_objective_value = sol.objective;
    ps.constraint_slack.reserve(SCHED_HOURS);
    for (int t = 0; t < SCHED_HOURS; ++t) ps.constraint_slack.push_back(sol.reserveSlack[t]);
    ps.note = sol.optimal ? "Optimal MIP solution" : (sol.feasible ? "Suboptimal solution" : "Infeasible");
}

float PlantScheduler::computeOptimizedEfficiency(const MILPSolution& sol) {
    float totalWeightedEff = 0.0f;
    float totalPow = 0.0f;
    for (int i = 0; i < SCHED_UNITS; ++i) {
        for (int t = 0; t < SCHED_HOURS; ++t) {
            float p = sol.power[i][t];
            if (p < 0.1f) continue;
            float eff = efficiencyCurves_[i].efficiency(p, headForecast_[t]);
            totalWeightedEff += eff * p;
            totalPow += p;
        }
    }
    return totalPow > 1e-3f ? (totalWeightedEff / totalPow) * 100.0f : 0.0f;
}

float PlantScheduler::computeCavitationReduction(const MILPSolution& sol,
                                                 const std::array<float, SCHED_UNITS>& curPow) {
    (void)curPow;
    float baselineCav = 0.0f;
    float optCav = 0.0f;
    for (int i = 0; i < SCHED_UNITS; ++i) {
        for (int t = 0; t < SCHED_HOURS; ++t) {
            float p = sol.power[i][t];
            if (p < 0.1f) continue;
            float refP = efficiencyCurves_[i].ratedPowerMW() * 0.8f;
            baselineCav += efficiencyCurves_[i].cavitationPenalty(refP, headForecast_[t]);
            optCav += efficiencyCurves_[i].cavitationPenalty(p, headForecast_[t]);
        }
    }
    return baselineCav > 1e-6f ? (baselineCav - optCav) / baselineCav * 100.0f : 0.0f;
}

void PlantScheduler::updateCavitationStates(const std::vector<CavitationState>& cavitationMsg) {
    std::lock_guard<std::mutex> lk(mutex_);
    uint64_t now = currentTimestampMs();
    for (const auto& msg : cavitationMsg) {
        if (msg.turbine_id < 1 || msg.turbine_id > SCHED_UNITS) continue;
        int idx = msg.turbine_id - 1;
        float intensity = msg.cavitation_intensity;
        float stageMult = 1.0f;
        switch (msg.cavitation_stage) {
            case CavitationStage::NORMAL:    stageMult = 1.0f; break;
            case CavitationStage::INCIPIENT: stageMult = 2.0f; break;
            case CavitationStage::CRITICAL:  stageMult = 4.0f; break;
            case CavitationStage::DEVELOPED: stageMult = 8.0f; break;
            default: break;
        }
        cavPenaltyMultiplier_[idx] = 1.0f + intensity * 8.0f * stageMult;
        lastCavityUpdateMs_[idx] = now;
    }
}

}
