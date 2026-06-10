#include "robot_planner.h"
#include <algorithm>
#include <cstring>

namespace robot_plan {

RobotPose DynamicPositioner::correct(const RobotPose& measured,
    const RobotPose& reference, const std::array<float, 3>& disturbance) {
    observer_.record(disturbance);
    auto w_est = observer_.estimate();
    std::array<float, 3> ctrl{};
    ctrl[0] = cfg_.kp * (reference.position.x - measured.position.x);
    ctrl[1] = cfg_.kp * (reference.position.y - measured.position.y);
    ctrl[2] = cfg_.kp * (reference.position.z - measured.position.z);
    if (cfg_.enabled) {
        for (int i = 0; i < 3; ++i) ctrl[i] += cfg_.feedforward_gain * w_est[i];
    }
    for (auto& v : ctrl) v = std::max(-cfg_.max_thrust_m, std::min(cfg_.max_thrust_m, v));
    RobotPose out = measured;
    out.position.x += ctrl[0];
    out.position.y += ctrl[1];
    out.position.z += ctrl[2];
    return out;
}

DamageHeatMap::DamageHeatMap() : damageGrid_(TOTAL_DAMAGE_GRIDS, 0.0f) {}

size_t DamageHeatMap::cellIndex(uint8_t turbine_id, uint8_t blade_id,
    uint8_t radial_idx, uint8_t angular_idx) const {
    return (static_cast<size_t>(turbine_id) * BLADES_PER_TURBINE + blade_id)
         * GRID_CELLS_PER_BLADE + radial_idx * ANGULAR_GRID_N + angular_idx;
}

void DamageHeatMap::update(uint8_t turbine_id, uint8_t blade_id,
    float cumulative_damage, float cavitation_intensity) {
    computeDamageDistribution(turbine_id, blade_id, cumulative_damage, cavitation_intensity);
}

std::array<float, GRID_CELLS_PER_BLADE> DamageHeatMap::getBladeDamageMap(
    uint8_t turbine_id, uint8_t blade_id) const {
    std::array<float, GRID_CELLS_PER_BLADE> m{};
    size_t base = cellIndex(turbine_id, blade_id, 0, 0);
    for (size_t i = 0; i < GRID_CELLS_PER_BLADE; ++i) m[i] = damageGrid_[base + i];
    return m;
}

float DamageHeatMap::getCellDamage(uint8_t turbine_id, uint8_t blade_id,
    uint8_t radial_idx, uint8_t angular_idx) const {
    return damageGrid_[cellIndex(turbine_id, blade_id, radial_idx, angular_idx)];
}

void DamageHeatMap::setCellDamage(uint8_t turbine_id, uint8_t blade_id,
    uint8_t radial_idx, uint8_t angular_idx, float value) {
    damageGrid_[cellIndex(turbine_id, blade_id, radial_idx, angular_idx)] = value;
}

void DamageHeatMap::computeDamageDistribution(uint8_t turbine_id, uint8_t blade_id,
    float cumulative_damage, float cavitation_intensity) {
    float center_peak = 1.0f + cavitation_intensity * 2.0f;
    float trailing_bias = 0.8f + cavitation_intensity;
    for (uint8_t ri = 0; ri < RADIAL_GRID_N; ++ri) {
        for (uint8_t ai = 0; ai < ANGULAR_GRID_N; ++ai) {
            float r_norm = static_cast<float>(ri) / (RADIAL_GRID_N - 1);
            float a_norm = static_cast<float>(ai) / (ANGULAR_GRID_N - 1);
            float radial_w = std::exp(-0.5f * (r_norm - 0.5f) * (r_norm - 0.5f) / 0.08f);
            float angular_w = std::exp(-0.5f * (a_norm - 0.7f) * (a_norm - 0.7f) / 0.12f);
            float val = cumulative_damage * center_peak * radial_w * (0.4f + 0.6f * angular_w * trailing_bias);
            setCellDamage(turbine_id, blade_id, ri, ai, val);
        }
    }
}

RRTStarPathPlanner::RRTStarPathPlanner() :
    r_inner_(R_INNER_MM), r_outer_(R_OUTER_MM), thickness_(BLADE_THICKNESS_MM),
    safety_dist_(SAFETY_DISTANCE_MM), max_iter_(300), step_size_(10.0f),
    rewire_radius_(RRT_REWIRE_RADIUS_MM), rng_(5678) {}

void RRTStarPathPlanner::setWorkspace(float ir, float or_, float t) {
    r_inner_ = ir; r_outer_ = or_; thickness_ = t;
}
void RRTStarPathPlanner::setSafetyDistance(float mm) { safety_dist_ = mm; }
void RRTStarPathPlanner::setMaxIterations(uint32_t it) { max_iter_ = it; }
void RRTStarPathPlanner::setStepSize(float mm) { step_size_ = mm; }

Point3D RRTStarPathPlanner::sampleFree() {
    std::uniform_real_distribution<float> r_dist(r_inner_ + safety_dist_, r_outer_ - safety_dist_);
    std::uniform_real_distribution<float> a_dist(0.0f, 2 * 3.14159265358979323846f);
    std::uniform_real_distribution<float> t_dist(-thickness_ / 2, thickness_ / 2);
    float r = r_dist(rng_), a = a_dist(rng_), t = t_dist(rng_);
    return {r * std::cos(a), r * std::sin(a), t};
}

int32_t RRTStarPathPlanner::nearest(const Point3D& p) {
    float best = 1e18f; int32_t idx = -1;
    for (size_t i = 0; i < nodes_.size(); ++i) {
        float d = nodes_[i].pose.position.distance(p);
        if (d < best) { best = d; idx = static_cast<int32_t>(i); }
    }
    return idx;
}

std::vector<int32_t> RRTStarPathPlanner::near(const Point3D& p, float radius) {
    std::vector<int32_t> out;
    for (size_t i = 0; i < nodes_.size(); ++i) {
        if (nodes_[i].pose.position.distance(p) < radius) out.push_back(static_cast<int32_t>(i));
    }
    return out;
}

RobotPose RRTStarPathPlanner::steer(const RobotPose& from, const Point3D& to, float step) {
    Point3D diff = to - from.position;
    float d = diff.distance({0, 0, 0});
    if (d < 1e-6f) return from;
    float scale = std::min(1.0f, step / d);
    RobotPose out = from;
    out.position = from.position + diff * scale;
    return out;
}

bool RRTStarPathPlanner::isInWorkspace(const Point3D& p) const {
    float r = std::sqrt(p.x * p.x + p.y * p.y);
    return r >= r_inner_ + safety_dist_ && r <= r_outer_ - safety_dist_
        && std::abs(p.z) <= thickness_ / 2 - safety_dist_;
}

bool RRTStarPathPlanner::checkCollisionLine(const Point3D& a, const Point3D& b,
    const std::function<bool(const Point3D&)>& constraints) {
    if (!isInWorkspace(a) || !isInWorkspace(b)) return false;
    int steps = 10;
    for (int i = 0; i <= steps; ++i) {
        float t = static_cast<float>(i) / steps;
        Point3D p = a + (b - a) * t;
        if (!isInWorkspace(p)) return false;
        if (constraints && !constraints(p)) return false;
    }
    return true;
}

float RRTStarPathPlanner::lineCost(const RobotPose& a, const RobotPose& b) {
    return a.position.distance(b.position);
}

RobotWaypoint RRTStarPathPlanner::poseToWaypoint(const RobotPose& pose, float speed) {
    RobotWaypoint wp;
    std::memset(&wp, 0, sizeof(wp));
    wp.x_mm = pose.position.x;
    wp.y_mm = pose.position.y;
    wp.z_mm = pose.position.z;
    wp.qw = pose.orientation.w; wp.qx = pose.orientation.x;
    wp.qy = pose.orientation.y; wp.qz = pose.orientation.z;
    wp.speed_mm_s = speed;
    return wp;
}

std::vector<Point3D> RRTStarPathPlanner::bspline3(const std::vector<Point3D>& cp, int samples) {
    std::vector<Point3D> out;
    if (cp.size() < 4) { for (const auto& p : cp) out.push_back(p); return out; }
    for (int s = 0; s < samples; ++s) {
        float u = static_cast<float>(s) / samples;
        for (size_t i = 0; i <= cp.size() - 4; ++i) {
            float b0 = (1 - u) * (1 - u) * (1 - u) / 6.0f;
            float b1 = (3 * u * u * u - 6 * u * u + 4) / 6.0f;
            float b2 = (-3 * u * u * u + 3 * u * u + 3 * u + 1) / 6.0f;
            float b3 = u * u * u / 6.0f;
            Point3D p = cp[i] * b0 + cp[i + 1] * b1 + cp[i + 2] * b2 + cp[i + 3] * b3;
            out.push_back(p);
        }
    }
    return out;
}

std::vector<RobotWaypoint> RRTStarPathPlanner::plan(
    const RobotPose& start, const RobotPose& goal,
    const std::function<bool(const Point3D&)>& constraints) {
    nodes_.clear();
    RRTNode s; s.pose = start; s.parent = -1; s.cost = 0; s.id = 0;
    nodes_.push_back(s);
    for (uint32_t iter = 0; iter < max_iter_; ++iter) {
        Point3D rand = sampleFree();
        int32_t ni = nearest(rand);
        if (ni < 0) continue;
        RobotPose new_pose = steer(nodes_[ni].pose, rand, step_size_);
        if (!checkCollisionLine(nodes_[ni].pose.position, new_pose.position, constraints)) continue;
        RRTNode nn; nn.pose = new_pose; nn.parent = ni;
        nn.cost = nodes_[ni].cost + lineCost(nodes_[ni].pose, new_pose);
        nn.id = static_cast<uint32_t>(nodes_.size());
        auto nears = near(new_pose.position, rewire_radius_);
        for (int32_t n : nears) {
            float alt = nodes_[n].cost + lineCost(nodes_[n].pose, new_pose);
            if (alt < nn.cost && checkCollisionLine(nodes_[n].pose.position, new_pose.position, constraints)) {
                nn.cost = alt; nn.parent = n;
            }
        }
        nodes_.push_back(nn);
        for (int32_t n : nears) {
            float alt = nn.cost + lineCost(new_pose, nodes_[n].pose);
            if (alt < nodes_[n].cost && checkCollisionLine(new_pose.position, nodes_[n].pose.position, constraints)) {
                nodes_[n].cost = alt; nodes_[n].parent = static_cast<int32_t>(nodes_.size() - 1);
            }
        }
    }
    std::vector<RobotWaypoint> out;
    if (nodes_.empty()) return out;
    int32_t best = -1; float best_cost = 1e18f;
    for (size_t i = 0; i < nodes_.size(); ++i) {
        float d = nodes_[i].pose.position.distance(goal.position);
        float c = nodes_[i].cost + d;
        if (c < best_cost) { best_cost = c; best = static_cast<int32_t>(i); }
    }
    std::vector<RobotPose> path;
    for (int32_t cur = best; cur >= 0; cur = nodes_[cur].parent) path.push_back(nodes_[cur].pose);
    std::reverse(path.begin(), path.end());
    path.push_back(goal);
    float speed = 5.0f;
    for (const auto& p : path) out.push_back(poseToWaypoint(p, speed));
    return out;
}

std::vector<RobotWaypoint> RRTStarPathPlanner::smooth(const std::vector<RobotWaypoint>& path, float) {
    if (path.size() < 3) return path;
    std::vector<RobotWaypoint> out;
    for (size_t i = 0; i < path.size(); ++i) {
        size_t i0 = std::max<size_t>(0, i - 1);
        size_t i1 = i;
        size_t i2 = std::min(path.size() - 1, i + 1);
        RobotWaypoint w;
        std::memset(&w, 0, sizeof(w));
        w.x_mm = (path[i0].x_mm + path[i1].x_mm + path[i2].x_mm) / 3.0f;
        w.y_mm = (path[i0].y_mm + path[i1].y_mm + path[i2].y_mm) / 3.0f;
        w.z_mm = (path[i0].z_mm + path[i1].z_mm + path[i2].z_mm) / 3.0f;
        w.qw = path[i1].qw; w.qx = path[i1].qx; w.qy = path[i1].qy; w.qz = path[i1].qz;
        w.speed_mm_s = path[i1].speed_mm_s;
        out.push_back(w);
    }
    return out;
}

WeldTrajectoryGenerator::WeldTrajectoryGenerator() :
    bead_width_(8.0f), overlap_ratio_(0.3f), weld_speed_(3.0f) {}

void WeldTrajectoryGenerator::setBeadWidth(float w) { bead_width_ = w; }
void WeldTrajectoryGenerator::setOverlapRatio(float r) { overlap_ratio_ = r; }
void WeldTrajectoryGenerator::setWeldSpeed(float s) { weld_speed_ = s; }

Point3D WeldTrajectoryGenerator::bladeSurfacePoint(uint8_t blade_id, float r, float z_offset) {
    float blade_angle = 2 * 3.14159265358979323846f * blade_id / BLADES_PER_TURBINE;
    float x = r * std::cos(blade_angle);
    float y = r * std::sin(blade_angle);
    return {x, y, z_offset};
}

Quaternion WeldTrajectoryGenerator::bladeNormalOrientation(uint8_t blade_id, float r) {
    float blade_angle = 2 * 3.14159265358979323846f * blade_id / BLADES_PER_TURBINE;
    float nx = -std::sin(blade_angle);
    float ny = std::cos(blade_angle);
    float axis_len = std::sqrt(nx * nx + ny * ny + 0.1f);
    return Quaternion::fromAngleAxis(0.0f, nx / axis_len, ny / axis_len, 0.1f / axis_len);
}

ContourPoint WeldTrajectoryGenerator::interpolate(float v0, float v1,
    const ContourPoint& p0, const ContourPoint& p1, float t) {
    float denom = v1 - v0;
    if (std::abs(denom) < 1e-6f) return p0;
    float a = (t - v0) / denom;
    ContourPoint p;
    p.r = p0.r + (p1.r - p0.r) * a;
    p.theta = p0.theta + (p1.theta - p0.theta) * a;
    return p;
}

void WeldTrajectoryGenerator::marchingSquaresCell(
    const std::array<float, GRID_CELLS_PER_BLADE>& grid,
    int ri, int ai, float t, std::vector<std::pair<ContourPoint, ContourPoint>>& segs) {
    auto at = [&](int r, int a) -> float {
        if (r < 0) r = 0; if (r >= RADIAL_GRID_N) r = RADIAL_GRID_N - 1;
        if (a < 0) a = 0; if (a >= ANGULAR_GRID_N) a = ANGULAR_GRID_N - 1;
        return grid[r * ANGULAR_GRID_N + a];
    };
    float v00 = at(ri, ai), v10 = at(ri + 1, ai), v01 = at(ri, ai + 1), v11 = at(ri + 1, ai + 1);
    ContourPoint p00(ri, ai), p10(ri + 1, ai), p01(ri, ai + 1), p11(ri + 1, ai + 1);
    int idx = (v00 > t ? 8 : 0) | (v10 > t ? 4 : 0) | (v11 > t ? 2 : 0) | (v01 > t ? 1 : 0);
    if (idx == 0 || idx == 15) return;
    auto left = interpolate(v00, v01, p00, p01, t);
    auto right = interpolate(v10, v11, p10, p11, t);
    auto top = interpolate(v00, v10, p00, p10, t);
    auto bottom = interpolate(v01, v11, p01, p11, t);
    switch (idx) {
        case 1: case 14: segs.push_back({left, bottom}); break;
        case 2: case 13: segs.push_back({bottom, right}); break;
        case 3: case 12: segs.push_back({left, right}); break;
        case 4: case 11: segs.push_back({top, right}); break;
        case 6: case 9: segs.push_back({top, bottom}); break;
        case 7: case 8: segs.push_back({left, top}); break;
        case 5: segs.push_back({left, top}); segs.push_back({bottom, right}); break;
        case 10: segs.push_back({left, bottom}); segs.push_back({top, right}); break;
    }
}

std::vector<std::vector<ContourPoint>> WeldTrajectoryGenerator::extractContours(
    const std::array<float, GRID_CELLS_PER_BLADE>& grid, float t) {
    std::vector<std::pair<ContourPoint, ContourPoint>> segs;
    for (int ri = 0; ri < RADIAL_GRID_N - 1; ++ri)
        for (int ai = 0; ai < ANGULAR_GRID_N - 1; ++ai)
            marchingSquaresCell(grid, ri, ai, t, segs);
    std::vector<std::vector<ContourPoint>> contours;
    std::vector<ContourPoint> cur;
    for (const auto& s : segs) {
        if (cur.empty()) { cur.push_back(s.first); cur.push_back(s.second); continue; }
        float d0 = std::hypot(cur.back().r - s.first.r, cur.back().theta - s.first.theta);
        float d1 = std::hypot(cur.back().r - s.second.r, cur.back().theta - s.second.theta);
        if (d0 < d1 && d0 < 2.0f) cur.push_back(s.second);
        else if (d1 < 2.0f) cur.push_back(s.first);
        else { contours.push_back(cur); cur.clear(); cur.push_back(s.first); cur.push_back(s.second); }
    }
    if (!cur.empty()) contours.push_back(cur);
    return contours;
}

std::vector<std::vector<ContourPoint>> WeldTrajectoryGenerator::segmentsToContours(
    const std::vector<std::pair<ContourPoint, ContourPoint>>& segs) {
    std::vector<std::vector<ContourPoint>> out;
    std::vector<bool> used(segs.size(), false);
    for (size_t i = 0; i < segs.size(); ++i) {
        if (used[i]) continue;
        std::vector<ContourPoint> c{segs[i].first, segs[i].second};
        used[i] = true;
        bool extended = true;
        while (extended) {
            extended = false;
            for (size_t j = 0; j < segs.size(); ++j) {
                if (used[j]) continue;
                auto b = c.back();
                float d0 = std::hypot(b.r - segs[j].first.r, b.theta - segs[j].first.theta);
                float d1 = std::hypot(b.r - segs[j].second.r, b.theta - segs[j].second.theta);
                if (d0 < 2.0f) { c.push_back(segs[j].second); used[j] = true; extended = true; }
                else if (d1 < 2.0f) { c.push_back(segs[j].first); used[j] = true; extended = true; }
            }
        }
        out.push_back(c);
    }
    return out;
}

std::vector<RobotWaypoint> WeldTrajectoryGenerator::contourToZigzagFill(
    const std::vector<ContourPoint>& contour, float line_spacing) {
    std::vector<RobotWaypoint> out;
    if (contour.size() < 2) return out;
    float r_min = 1e9f, r_max = -1e9f;
    for (const auto& p : contour) { if (p.r < r_min) r_min = p.r; if (p.r > r_max) r_max = p.r; }
    bool zig = true;
    for (float r = r_min; r <= r_max; r += line_spacing) {
        RobotWaypoint wp; std::memset(&wp, 0, sizeof(wp));
        if (zig) {
            wp.x_mm = r * 10.0f; wp.y_mm = 0;
        } else {
            wp.x_mm = r * 10.0f; wp.y_mm = 50.0f;
        }
        zig = !zig;
        wp.z_mm = 0; wp.qw = 1; wp.speed_mm_s = weld_speed_;
        out.push_back(wp);
    }
    return out;
}

std::vector<RobotWaypoint> WeldTrajectoryGenerator::generate(
    const std::array<float, GRID_CELLS_PER_BLADE>& damage_map,
    uint8_t turbine_id, uint8_t blade_id, float threshold) {
    auto contours = extractContours(damage_map, threshold);
    std::vector<RobotWaypoint> out;
    float spacing = bead_width_ * (1.0f - overlap_ratio_);
    for (const auto& c : contours) {
        auto wps = contourToZigzagFill(c, spacing);
        for (auto& w : wps) {
            auto p = bladeSurfacePoint(blade_id,
                std::sqrt(w.x_mm * w.x_mm + w.y_mm * w.y_mm) / 10.0f, w.z_mm);
            auto q = bladeNormalOrientation(blade_id,
                std::sqrt(w.x_mm * w.x_mm + w.y_mm * w.y_mm) / 10.0f);
            w.x_mm = p.x; w.y_mm = p.y; w.z_mm = p.z;
            w.qw = q.w; w.qx = q.x; w.qy = q.y; w.qz = q.z;
            (void)turbine_id;
        }
        out.insert(out.end(), wps.begin(), wps.end());
    }
    return out;
}

PolishTrajectoryGenerator::PolishTrajectoryGenerator() :
    grid_step_(10.0f), polish_speed_(5.0f), normal_force_(2.0f) {}

void PolishTrajectoryGenerator::setGridStep(float s) { grid_step_ = s; }
void PolishTrajectoryGenerator::setPolishSpeed(float s) { polish_speed_ = s; }
void PolishTrajectoryGenerator::setForceNormal(float f) { normal_force_ = f; }

Point3D PolishTrajectoryGenerator::bladeSurfacePoint(uint8_t blade_id, float r, float theta_local, float z_offset) {
    float base = 2 * 3.14159265358979323846f * blade_id / BLADES_PER_TURBINE + theta_local;
    return {r * std::cos(base), r * std::sin(base), z_offset};
}

Quaternion PolishTrajectoryGenerator::bladeNormalOrientation(uint8_t blade_id, float r, float theta_local) {
    float base = 2 * 3.14159265358979323846f * blade_id / BLADES_PER_TURBINE + theta_local;
    float ta = twistAngle(r / R_OUTER_MM);
    float nx = -std::sin(base) * std::cos(ta);
    float ny = std::cos(base) * std::cos(ta);
    float nz = std::sin(ta);
    return Quaternion::fromAngleAxis(0.0f, nx, ny, nz);
}

float PolishTrajectoryGenerator::twistAngle(float r_norm) {
    return -TWIST_RAD_MAX * std::pow(r_norm, 1.5f);
}

std::vector<RobotWaypoint> PolishTrajectoryGenerator::generate(
    const std::array<float, GRID_CELLS_PER_BLADE>& damage_map,
    uint8_t turbine_id, uint8_t blade_id, float threshold) {
    std::vector<RobotWaypoint> out;
    for (uint8_t ri = 0; ri < RADIAL_GRID_N; ++ri) {
        for (uint8_t ai = 0; ai < ANGULAR_GRID_N; ++ai) {
            if (damage_map[ri * ANGULAR_GRID_N + ai] < threshold) continue;
            float r = R_INNER_MM + (R_OUTER_MM - R_INNER_MM) * ri / (RADIAL_GRID_N - 1);
            float theta_local = -0.3f + 0.6f * ai / (ANGULAR_GRID_N - 1);
            auto p = bladeSurfacePoint(blade_id, r, theta_local, 0);
            auto q = bladeNormalOrientation(blade_id, r, theta_local);
            RobotWaypoint wp;
            std::memset(&wp, 0, sizeof(wp));
            wp.x_mm = p.x; wp.y_mm = p.y; wp.z_mm = p.z;
            wp.qw = q.w; wp.qx = q.x; wp.qy = q.y; wp.qz = q.z;
            wp.speed_mm_s = polish_speed_;
            (void)turbine_id;
            out.push_back(wp);
        }
    }
    return out;
}

RobotPlannerFacade::RobotPlannerFacade() {
    rrt_planner_.setMaxIterations(300);
}

void RobotPlannerFacade::setConfig(const Config::RobotConfig& cfg) { config_ = cfg; }

bool RobotPlannerFacade::autoTriggerCheck(const std::vector<LifeAssessment>& data) {
    for (const auto& d : data) if (d.remaining_life_pct < config_.life_threshold_pct) return true;
    return false;
}

RobotPose RobotPlannerFacade::computeBladeInspectionPose(uint8_t blade_id, uint8_t waypoint_idx) {
    float r = R_INNER_MM + (R_OUTER_MM - R_INNER_MM) * waypoint_idx / 4.0f;
    float a = 2 * 3.14159265358979323846f * blade_id / BLADES_PER_TURBINE;
    RobotPose p;
    p.position = {r * std::cos(a), r * std::sin(a), 0};
    p.orientation = Quaternion::fromAngleAxis(a, 0, 0, 1);
    return p;
}

std::vector<uint8_t> RobotPlannerFacade::selectBladesByDamage(uint8_t turbine_id, float min_damage) {
    std::vector<uint8_t> out;
    for (uint8_t b = 0; b < BLADES_PER_TURBINE; ++b) {
        auto m = heatMap_.getBladeDamageMap(turbine_id, b);
        float max_v = *std::max_element(m.begin(), m.end());
        if (max_v >= min_damage) out.push_back(b);
    }
    return out;
}

float RobotPlannerFacade::estimateTaskDuration(const RobotRepairTask& task) {
    float total_mm = 0;
    for (size_t i = 1; i < task.waypoint_count; ++i) {
        float dx = task.waypoints[i].x_mm - task.waypoints[i - 1].x_mm;
        float dy = task.waypoints[i].y_mm - task.waypoints[i - 1].y_mm;
        float dz = task.waypoints[i].z_mm - task.waypoints[i - 1].z_mm;
        total_mm += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    float speed = 10.0f;
    return total_mm / speed / 1000.0f;
}

float RobotPlannerFacade::computeRepairArea(const std::array<float, GRID_CELLS_PER_BLADE>& m, float t) {
    float count = 0;
    for (auto v : m) if (v >= t) count++;
    float cell = (R_OUTER_MM - R_INNER_MM) / RADIAL_GRID_N * (R_OUTER_MM / ANGULAR_GRID_N);
    return count * cell / 100.0f;
}

float RobotPlannerFacade::computeWeldVolume(float area_cm2, float avg_depth_mm) {
    return area_cm2 * avg_depth_mm * 0.1f;
}

RobotRepairTask RobotPlannerFacade::planFullInspection(uint8_t turbine_id) {
    RobotRepairTask task;
    std::memset(&task, 0, sizeof(task));
    task.task_id = 1;
    task.turbine_id = turbine_id;
    task.mode = RepairMode::INSPECT;
    task.status = RepairStatus::PLANNED;
    size_t idx = 0;
    for (uint8_t b = 0; b < BLADES_PER_TURBINE && idx < MAX_ROBOT_WAYPOINTS; ++b) {
        for (uint8_t w = 0; w < 5 && idx < MAX_ROBOT_WAYPOINTS; ++w) {
            auto pose = computeBladeInspectionPose(b, w);
            task.waypoints[idx].x_mm = pose.position.x;
            task.waypoints[idx].y_mm = pose.position.y;
            task.waypoints[idx].z_mm = pose.position.z;
            task.waypoints[idx].qw = pose.orientation.w;
            task.waypoints[idx].qx = pose.orientation.x;
            task.waypoints[idx].qy = pose.orientation.y;
            task.waypoints[idx].qz = pose.orientation.z;
            task.waypoints[idx].speed_mm_s = 10.0f;
            idx++;
        }
    }
    task.waypoint_count = static_cast<uint32_t>(idx);
    task.estimated_duration_min = static_cast<uint32_t>(estimateTaskDuration(task));
    return task;
}

RobotRepairTask RobotPlannerFacade::planRepair(uint8_t turbine_id,
    const std::vector<uint8_t>& blade_ids, RepairMode repair_mode) {
    RobotRepairTask task;
    std::memset(&task, 0, sizeof(task));
    task.task_id = 2;
    task.turbine_id = turbine_id;
    task.mode = repair_mode;
    task.status = RepairStatus::PLANNED;
    size_t idx = 0;
    RobotPose start({R_OUTER_MM + 100, 0, 0}, Quaternion());
    RobotPose goal({R_OUTER_MM + 100, 0, 0}, Quaternion());
    for (uint8_t b : blade_ids) {
        if (idx >= MAX_ROBOT_WAYPOINTS) break;
        auto damage = heatMap_.getBladeDamageMap(turbine_id, b);
        std::vector<RobotWaypoint> wps;
        if (repair_mode == RepairMode::WELD)
            wps = weld_gen_.generate(damage, turbine_id, b);
        else
            wps = polish_gen_.generate(damage, turbine_id, b);
        auto start_pose = computeBladeInspectionPose(b, 0);
        auto path = rrt_planner_.plan(start, start_pose, nullptr);
        for (const auto& wp : path) if (idx < MAX_ROBOT_WAYPOINTS) task.waypoints[idx++] = wp;
        for (const auto& wp : wps) if (idx < MAX_ROBOT_WAYPOINTS) task.waypoints[idx++] = wp;
        start = goal;
    }
    task.waypoint_count = static_cast<uint32_t>(idx);
    task.estimated_duration_min = static_cast<uint32_t>(estimateTaskDuration(task));
    return task;
}

RobotWaypoint RobotPlannerFacade::simulateExecution(RobotRepairTask& task, uint32_t time_ms) {
    RobotWaypoint wp;
    std::memset(&wp, 0, sizeof(wp));
    float t_sec = time_ms / 1000.0f;
    float dist_acc = 0, speed = 10.0f;
    float target = t_sec * speed;
    for (uint32_t i = 1; i < task.waypoint_count; ++i) {
        float dx = task.waypoints[i].x_mm - task.waypoints[i - 1].x_mm;
        float dy = task.waypoints[i].y_mm - task.waypoints[i - 1].y_mm;
        float dz = task.waypoints[i].z_mm - task.waypoints[i - 1].z_mm;
        float seg = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist_acc + seg >= target) {
            float a = (target - dist_acc) / std::max(seg, 1e-6f);
            wp = task.waypoints[i - 1];
            wp.x_mm += dx * a; wp.y_mm += dy * a; wp.z_mm += dz * a;
            task.progress_pct = std::min(100.0f, 100.0f * target / std::max(dist_acc + seg, 1.0f));
            if (task.progress_pct >= 100.0f) task.status = RepairStatus::COMPLETED;
            return wp;
        }
        dist_acc += seg;
    }
    if (task.waypoint_count > 0) wp = task.waypoints[task.waypoint_count - 1];
    task.progress_pct = 100.0f;
    task.status = RepairStatus::COMPLETED;
    return wp;
}

}
