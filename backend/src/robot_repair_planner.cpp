#include "../include/robot_repair_planner.h"
#include <algorithm>
#include <map>
#include <set>
#include <unordered_map>
#include <cassert>
#include <numeric>

namespace turbine_monitor {

static constexpr float PI = 3.14159265358979323846f;
static constexpr float TWO_PI = 2.0f * PI;
static constexpr float BLADE_ANGLE_SPAN = TWO_PI / BLADES_PER_TURBINE;
static constexpr float WELD_DEPTH_MM = 3.0f;

static size_t bladeGridIndex(uint8_t ri, uint8_t ai) {
    return static_cast<size_t>(ri) * ANGULAR_GRID_N + ai;
}

static float radialIndexToRadius(uint8_t ri) {
    float t = (ri + 0.5f) / RADIAL_GRID_N;
    return R_INNER_MM + t * (R_OUTER_MM - R_INNER_MM);
}

static float angularIndexToThetaLocal(uint8_t ai) {
    return (ai + 0.5f) / ANGULAR_GRID_N * BLADE_ANGLE_SPAN;
}

static float bladeCenterAngle(uint8_t blade_id) {
    return (static_cast<float>(blade_id - 1) + 0.5f) / BLADES_PER_TURBINE * TWO_PI;
}

static Point3D polarToCartesian(float r, float theta_global, float z) {
    return Point3D(r * std::cos(theta_global), r * std::sin(theta_global), z);
}

DamageHeatMap::DamageHeatMap() {
    damageGrid_.assign(TOTAL_DAMAGE_GRIDS, 0.0f);
}

size_t DamageHeatMap::cellIndex(uint8_t turbine_id, uint8_t blade_id,
                                 uint8_t radial_idx, uint8_t angular_idx) const {
    size_t t = static_cast<size_t>(turbine_id - 1) * BLADES_PER_TURBINE * GRID_CELLS_PER_BLADE;
    size_t b = static_cast<size_t>(blade_id - 1) * GRID_CELLS_PER_BLADE;
    return t + b + bladeGridIndex(radial_idx, angular_idx);
}

void DamageHeatMap::update(uint8_t turbine_id, uint8_t blade_id,
                            float cumulative_damage, float cavitation_intensity) {
    computeDamageDistribution(turbine_id, blade_id, cumulative_damage, cavitation_intensity);
}

std::array<float, GRID_CELLS_PER_BLADE> DamageHeatMap::getBladeDamageMap(
    uint8_t turbine_id, uint8_t blade_id) const {
    std::array<float, GRID_CELLS_PER_BLADE> result{};
    size_t base = cellIndex(turbine_id, blade_id, 0, 0);
    for (size_t i = 0; i < GRID_CELLS_PER_BLADE; ++i) {
        result[i] = damageGrid_[base + i];
    }
    return result;
}

float DamageHeatMap::getCellDamage(uint8_t turbine_id, uint8_t blade_id,
                                    uint8_t radial_idx, uint8_t angular_idx) const {
    return damageGrid_[cellIndex(turbine_id, blade_id, radial_idx, angular_idx)];
}

void DamageHeatMap::setCellDamage(uint8_t turbine_id, uint8_t blade_id,
                                   uint8_t radial_idx, uint8_t angular_idx, float value) {
    damageGrid_[cellIndex(turbine_id, blade_id, radial_idx, angular_idx)] =
        std::clamp(value, 0.0f, 1.0f);
}

void DamageHeatMap::computeDamageDistribution(uint8_t turbine_id, uint8_t blade_id,
                                               float cumulative_damage, float cavitation_intensity) {
    float base_damage = std::clamp(cumulative_damage, 0.0f, 1.0f);
    float cav_weight = std::clamp(cavitation_intensity, 0.0f, 1.0f);

    float r_center = (R_INNER_MM + R_OUTER_MM) * 0.5f;
    float r_range = R_OUTER_MM - R_INNER_MM;
    float theta_cent = BLADE_ANGLE_SPAN * 0.5f;

    for (uint8_t ri = 0; ri < RADIAL_GRID_N; ++ri) {
        float r = radialIndexToRadius(ri);
        float radial_factor = 1.0f - std::abs(r - r_center) / (r_range * 0.5f);
        radial_factor = 0.4f + 0.6f * std::max(0.0f, radial_factor);

        for (uint8_t ai = 0; ai < ANGULAR_GRID_N; ++ai) {
            float theta_local = angularIndexToThetaLocal(ai);
            float angular_factor = 1.0f - std::abs(theta_local - theta_cent) / (BLADE_ANGLE_SPAN * 0.5f);
            angular_factor = 0.5f + 0.5f * std::max(0.0f, angular_factor);

            float tip_factor = (r - R_INNER_MM) / r_range;
            float cavitation_bias = cav_weight * (0.3f + 0.7f * tip_factor);

            float cell_value = base_damage * radial_factor * angular_factor
                               + cavitation_bias * 0.6f;

            float noise = 0.0f;
            uint32_t seed = (turbine_id * 131 + blade_id * 17 + ri * 7 + ai) * 2654435761u;
            noise = ((seed & 0xFFFF) / 65535.0f - 0.5f) * 0.05f;

            setCellDamage(turbine_id, blade_id, ri, ai, cell_value + noise);
        }
    }
}

RRTStarPathPlanner::RRTStarPathPlanner()
    : r_inner_(R_INNER_MM), r_outer_(R_OUTER_MM), thickness_(BLADE_THICKNESS_MM),
      safety_dist_(SAFETY_DISTANCE_MM), max_iter_(2000), step_size_(100.0f),
      rewire_radius_(RRT_REWIRE_RADIUS_MM * 5.0f), rng_(42) {}

void RRTStarPathPlanner::setWorkspace(float inner_radius_mm, float outer_radius_mm, float thickness_mm) {
    r_inner_ = inner_radius_mm;
    r_outer_ = outer_radius_mm;
    thickness_ = thickness_mm;
}

void RRTStarPathPlanner::setSafetyDistance(float mm) { safety_dist_ = mm; }
void RRTStarPathPlanner::setMaxIterations(uint32_t iter) { max_iter_ = iter; }
void RRTStarPathPlanner::setStepSize(float mm) { step_size_ = mm; }

Point3D RRTStarPathPlanner::sampleFree() {
    std::uniform_real_distribution<float> r_dist(r_inner_ + safety_dist_, r_outer_ - safety_dist_);
    std::uniform_real_distribution<float> theta_dist(0.0f, TWO_PI);
    std::uniform_real_distribution<float> z_dist(-thickness_ * 0.5f + safety_dist_,
                                                  thickness_ * 0.5f - safety_dist_);
    float r = r_dist(rng_);
    float theta = theta_dist(rng_);
    float z = z_dist(rng_);
    return polarToCartesian(r, theta, z);
}

int32_t RRTStarPathPlanner::nearest(const Point3D& p) {
    int32_t best = -1;
    float best_d = 1e20f;
    for (size_t i = 0; i < nodes_.size(); ++i) {
        float d = nodes_[i].pose.position.distance(p);
        if (d < best_d) { best_d = d; best = static_cast<int32_t>(i); }
    }
    return best;
}

std::vector<int32_t> RRTStarPathPlanner::near(const Point3D& p, float radius) {
    std::vector<int32_t> result;
    float r2 = radius * radius;
    for (size_t i = 0; i < nodes_.size(); ++i) {
        float dx = nodes_[i].pose.position.x - p.x;
        float dy = nodes_[i].pose.position.y - p.y;
        float dz = nodes_[i].pose.position.z - p.z;
        if (dx*dx + dy*dy + dz*dz <= r2) {
            result.push_back(static_cast<int32_t>(i));
        }
    }
    return result;
}

bool RRTStarPathPlanner::isInWorkspace(const Point3D& p) const {
    float r = std::sqrt(p.x * p.x + p.y * p.y);
    if (r < r_inner_ + safety_dist_ || r > r_outer_ - safety_dist_) return false;
    if (std::abs(p.z) > thickness_ * 0.5f - safety_dist_) return false;
    return true;
}

RobotPose RRTStarPathPlanner::steer(const RobotPose& from, const Point3D& to, float step) {
    Point3D dir = to - from.position;
    float dist = dir.distance(Point3D(0,0,0));
    RobotPose result = from;
    if (dist < 1e-6f) return result;
    float s = std::min(step, dist);
    dir = dir * (s / dist);
    result.position = from.position + dir;

    Point3D look = dir * (1.0f / std::max(dist, 1e-6f));
    float yaw = std::atan2(look.y, look.x);
    float pitch = -std::asin(std::clamp(look.z, -1.0f, 1.0f));
    Quaternion qy = Quaternion::fromAngleAxis(yaw, 0, 0, 1);
    Quaternion qp = Quaternion::fromAngleAxis(pitch, 0, 1, 0);
    result.orientation.w = qy.w * qp.w - qy.x * qp.x - qy.y * qp.y - qy.z * qp.z;
    result.orientation.x = qy.w * qp.x + qy.x * qp.w + qy.y * qp.z - qy.z * qp.y;
    result.orientation.y = qy.w * qp.y - qy.x * qp.z + qy.y * qp.w + qy.z * qp.x;
    result.orientation.z = qy.w * qp.z + qy.x * qp.y - qy.y * qp.x + qy.z * qp.w;
    result.orientation.normalize();
    return result;
}

bool RRTStarPathPlanner::checkCollisionLine(const Point3D& a, const Point3D& b,
    const std::function<bool(const Point3D&)>& constraints) {
    float len = a.distance(b);
    if (len < 1e-6f) return !isInWorkspace(a) || (constraints && !constraints(a));
    int steps = static_cast<int>(std::ceil(len / (safety_dist_ * 0.5f)));
    for (int s = 0; s <= steps; ++s) {
        float t = static_cast<float>(s) / steps;
        Point3D p(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
        if (!isInWorkspace(p)) return true;
        if (constraints && !constraints(p)) return true;
    }
    return false;
}

float RRTStarPathPlanner::lineCost(const RobotPose& a, const RobotPose& b) {
    return a.position.distance(b.position);
}

RobotWaypoint RRTStarPathPlanner::poseToWaypoint(const RobotPose& pose, float speed_mm_s) {
    RobotWaypoint wp{};
    wp.x = pose.position.x;
    wp.y = pose.position.y;
    wp.z = pose.position.z;
    wp.orientation_w = pose.orientation.w;
    wp.orientation_x = pose.orientation.x;
    wp.orientation_y = pose.orientation.y;
    wp.orientation_z = pose.orientation.z;
    wp.speed_mm_s = speed_mm_s;
    wp.dwell_time_s = 0.0f;
    wp.action_type = 0;
    return wp;
}

std::vector<Point3D> RRTStarPathPlanner::bspline3(const std::vector<Point3D>& cps, int samples) {
    std::vector<Point3D> result;
    int n = static_cast<int>(cps.size());
    if (n < 4) {
        for (auto& p : cps) result.push_back(p);
        return result;
    }
    auto basis = [](int i, float t) -> float {
        switch (i) {
            case 0: return (1.0f - t) * (1.0f - t) * (1.0f - t) / 6.0f;
            case 1: return (3.0f * t*t*t - 6.0f*t*t + 4.0f) / 6.0f;
            case 2: return (-3.0f*t*t*t + 3.0f*t*t + 3.0f*t + 1.0f) / 6.0f;
            case 3: return t * t * t / 6.0f;
            default: return 0.0f;
        }
    };
    for (int seg = 0; seg <= n - 4; ++seg) {
        for (int s = 0; s <= samples; ++s) {
            float t = (seg == n - 4 && s == samples) ? 1.0f : static_cast<float>(s) / samples;
            Point3D p;
            for (int k = 0; k < 4; ++k) {
                float b = basis(k, t);
                p.x += cps[seg + k].x * b;
                p.y += cps[seg + k].y * b;
                p.z += cps[seg + k].z * b;
            }
            if (s == 0 && seg > 0) continue;
            result.push_back(p);
        }
    }
    return result;
}

std::vector<RobotWaypoint> RRTStarPathPlanner::plan(
    const RobotPose& start, const RobotPose& goal,
    const std::function<bool(const Point3D&)>& blade_surface_constraints) {
    nodes_.clear();
    RRTNode startNode;
    startNode.pose = start;
    startNode.parent = -1;
    startNode.cost = 0.0f;
    startNode.id = 0;
    nodes_.push_back(startNode);

    int32_t goal_idx = -1;
    float goal_best_cost = 1e20f;

    for (uint32_t iter = 0; iter < max_iter_; ++iter) {
        std::uniform_real_distribution<float> goal_bias(0.0f, 1.0f);
        Point3D sample = (goal_bias(rng_) < 0.1f) ? goal.position : sampleFree();

        int32_t n_idx = nearest(sample);
        if (n_idx < 0) continue;

        RobotPose new_pose = steer(nodes_[n_idx].pose, sample, step_size_);

        if (checkCollisionLine(nodes_[n_idx].pose.position, new_pose.position, blade_surface_constraints))
            continue;

        RRTNode new_node;
        new_node.pose = new_pose;
        new_node.id = static_cast<uint32_t>(nodes_.size());
        new_node.parent = n_idx;
        new_node.cost = nodes_[n_idx].cost + lineCost(nodes_[n_idx].pose, new_pose);

        auto neighbors = near(new_pose.position, rewire_radius_);
        for (int32_t nb : neighbors) {
            if (nb == n_idx) continue;
            if (checkCollisionLine(nodes_[nb].pose.position, new_pose.position, blade_surface_constraints))
                continue;
            float c = nodes_[nb].cost + lineCost(nodes_[nb].pose, new_pose);
            if (c < new_node.cost) {
                new_node.cost = c;
                new_node.parent = nb;
            }
        }
        nodes_.push_back(new_node);
        int32_t new_idx = static_cast<int32_t>(nodes_.size()) - 1;

        for (int32_t nb : neighbors) {
            if (nb == new_node.parent) continue;
            if (checkCollisionLine(new_pose.position, nodes_[nb].pose.position, blade_surface_constraints))
                continue;
            float c = new_node.cost + lineCost(new_pose, nodes_[nb].pose);
            if (c < nodes_[nb].cost) {
                nodes_[nb].parent = new_idx;
                nodes_[nb].cost = c;
            }
        }

        float gd = new_pose.position.distance(goal.position);
        if (gd < step_size_ * 0.5f && new_node.cost < goal_best_cost) {
            if (!checkCollisionLine(new_pose.position, goal.position, blade_surface_constraints)) {
                RRTNode goal_node;
                goal_node.pose = goal;
                goal_node.parent = new_idx;
                goal_node.cost = new_node.cost + gd;
                goal_node.id = static_cast<uint32_t>(nodes_.size());
                nodes_.push_back(goal_node);
                goal_idx = static_cast<int32_t>(nodes_.size()) - 1;
                goal_best_cost = goal_node.cost;
                break;
            }
        }
    }

    std::vector<RobotWaypoint> result;
    if (goal_idx < 0) {
        int32_t best = 0;
        float best_d = nodes_[0].pose.position.distance(goal.position);
        for (size_t i = 1; i < nodes_.size(); ++i) {
            float d = nodes_[i].pose.position.distance(goal.position);
            if (d < best_d) { best_d = d; best = static_cast<int32_t>(i); }
        }
        goal_idx = best;
    }

    std::vector<RobotPose> path;
    for (int32_t cur = goal_idx; cur >= 0; cur = nodes_[cur].parent) {
        path.push_back(nodes_[cur].pose);
    }
    std::reverse(path.begin(), path.end());

    for (auto& pose : path) {
        result.push_back(poseToWaypoint(pose, 50.0f));
    }
    return result;
}

std::vector<RobotWaypoint> RRTStarPathPlanner::smooth(
    const std::vector<RobotWaypoint>& path, float control_interval_mm) {
    if (path.size() < 4) return path;

    std::vector<Point3D> cps;
    for (auto& wp : path) cps.push_back(Point3D(wp.x, wp.y, wp.z));

    float total_len = 0.0f;
    for (size_t i = 1; i < cps.size(); ++i)
        total_len += cps[i-1].distance(cps[i]);
    int samples_per_seg = std::max(2, static_cast<int>(std::ceil(
        (control_interval_mm > 1e-6f) ? total_len / (control_interval_mm * std::max<size_t>(1, cps.size()-3)) : 10.0f)));

    auto smooth_pts = bspline3(cps, samples_per_seg);

    std::vector<RobotWaypoint> result;
    result.reserve(smooth_pts.size());
    for (size_t i = 0; i < smooth_pts.size(); ++i) {
        RobotWaypoint wp{};
        wp.x = smooth_pts[i].x;
        wp.y = smooth_pts[i].y;
        wp.z = smooth_pts[i].z;
        wp.speed_mm_s = 50.0f;

        Point3D dir;
        if (i == 0 && smooth_pts.size() > 1)
            dir = smooth_pts[1] - smooth_pts[0];
        else if (i > 0)
            dir = smooth_pts[i] - smooth_pts[i-1];
        float dl = dir.distance(Point3D(0,0,0));
        if (dl > 1e-6f) {
            dir = dir * (1.0f / dl);
            float yaw = std::atan2(dir.y, dir.x);
            float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
            wp.orientation_w = cy;
            wp.orientation_x = 0;
            wp.orientation_y = 0;
            wp.orientation_z = sy;
        } else {
            wp.orientation_w = 1;
        }
        result.push_back(wp);
    }
    return result;
}

WeldTrajectoryGenerator::WeldTrajectoryGenerator()
    : bead_width_(6.0f), overlap_ratio_(0.3f), weld_speed_(8.0f) {}

void WeldTrajectoryGenerator::setBeadWidth(float width_mm) { bead_width_ = width_mm; }
void WeldTrajectoryGenerator::setOverlapRatio(float ratio) { overlap_ratio_ = ratio; }
void WeldTrajectoryGenerator::setWeldSpeed(float speed_mm_s) { weld_speed_ = speed_mm_s; }

Point3D WeldTrajectoryGenerator::bladeSurfacePoint(uint8_t blade_id, float r, float z_offset) {
    float theta_center = bladeCenterAngle(blade_id);
    return polarToCartesian(r, theta_center, z_offset);
}

Quaternion WeldTrajectoryGenerator::bladeNormalOrientation(uint8_t blade_id, float r) {
    float theta_center = bladeCenterAngle(blade_id);
    float r_norm = (r - R_INNER_MM) / (R_OUTER_MM - R_INNER_MM);
    float twist = TWIST_RAD_MAX * r_norm;
    float nx = std::cos(theta_center + PI * 0.5f) * std::cos(twist);
    float ny = std::sin(theta_center + PI * 0.5f) * std::cos(twist);
    float nz = std::sin(twist);
    float nlen = std::sqrt(nx*nx + ny*ny + nz*nz);
    if (nlen > 1e-6f) { nx /= nlen; ny /= nlen; nz /= nlen; }

    float upx = 0, upy = 0, upz = 1;
    float tx = upy * nz - upz * ny;
    float ty = upz * nx - upx * nz;
    float tz = upx * ny - upy * nx;
    float tlen = std::sqrt(tx*tx + ty*ty + tz*tz);
    if (tlen > 1e-6f) { tx /= tlen; ty /= tlen; tz /= tlen; }
    float bx = ny * tz - nz * ty;
    float by = nz * tx - nx * tz;
    float bz = nx * ty - ny * tx;

    float m00 = tx, m01 = bx, m02 = nx;
    float m10 = ty, m11 = by, m12 = ny;
    float m20 = tz, m21 = bz, m22 = nz;

    float tr = m00 + m11 + m22;
    Quaternion q;
    if (tr > 0) {
        float s = std::sqrt(tr + 1.0f) * 2.0f;
        q.w = 0.25f * s;
        q.x = (m21 - m12) / s;
        q.y = (m02 - m20) / s;
        q.z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        float s = std::sqrt(1.0f + m00 - m11 - m22) * 2;
        q.w = (m21 - m12) / s;
        q.x = 0.25f * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
    } else if (m11 > m22) {
        float s = std::sqrt(1.0f + m11 - m00 - m22) * 2;
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = 0.25f * s;
        q.z = (m12 + m21) / s;
    } else {
        float s = std::sqrt(1.0f + m22 - m00 - m11) * 2;
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25f * s;
    }
    q.normalize();
    return q;
}

ContourPoint WeldTrajectoryGenerator::interpolate(float v0, float v1,
    const ContourPoint& p0, const ContourPoint& p1, float threshold) {
    float d = v1 - v0;
    if (std::abs(d) < 1e-6f) return p0;
    float t = (threshold - v0) / d;
    t = std::clamp(t, 0.0f, 1.0f);
    return ContourPoint(p0.r + t * (p1.r - p0.r),
                        p0.theta + t * (p1.theta - p0.theta));
}

void WeldTrajectoryGenerator::marchingSquaresCell(
    const std::array<float, GRID_CELLS_PER_BLADE>& grid,
    int ri, int ai, float threshold,
    std::vector<std::pair<ContourPoint, ContourPoint>>& segments) {
    if (ri < 0 || ri >= RADIAL_GRID_N - 1 || ai < 0 || ai >= ANGULAR_GRID_N - 1) return;

    ContourPoint p00(radialIndexToRadius(ri), angularIndexToThetaLocal(ai));
    ContourPoint p10(radialIndexToRadius(ri + 1), angularIndexToThetaLocal(ai));
    ContourPoint p11(radialIndexToRadius(ri + 1), angularIndexToThetaLocal(ai + 1));
    ContourPoint p01(radialIndexToRadius(ri), angularIndexToThetaLocal(ai + 1));

    float v00 = grid[bladeGridIndex(ri, ai)] - threshold;
    float v10 = grid[bladeGridIndex(ri + 1, ai)] - threshold;
    float v11 = grid[bladeGridIndex(ri + 1, ai + 1)] - threshold;
    float v01 = grid[bladeGridIndex(ri, ai + 1)] - threshold;

    int idx = 0;
    if (v00 > 0) idx |= 1;
    if (v10 > 0) idx |= 2;
    if (v11 > 0) idx |= 4;
    if (v01 > 0) idx |= 8;

    ContourPoint e_bottom = interpolate(grid[bladeGridIndex(ri, ai)],
                                         grid[bladeGridIndex(ri+1, ai)], p00, p10, threshold);
    ContourPoint e_right  = interpolate(grid[bladeGridIndex(ri+1, ai)],
                                         grid[bladeGridIndex(ri+1, ai+1)], p10, p11, threshold);
    ContourPoint e_top    = interpolate(grid[bladeGridIndex(ri, ai+1)],
                                         grid[bladeGridIndex(ri+1, ai+1)], p01, p11, threshold);
    ContourPoint e_left   = interpolate(grid[bladeGridIndex(ri, ai)],
                                         grid[bladeGridIndex(ri, ai+1)], p00, p01, threshold);

    switch (idx) {
        case 0: case 15: break;
        case 1: case 14: segments.push_back({e_left, e_bottom}); break;
        case 2: case 13: segments.push_back({e_bottom, e_right}); break;
        case 3: case 12: segments.push_back({e_left, e_right}); break;
        case 4: case 11: segments.push_back({e_right, e_top}); break;
        case 5: segments.push_back({e_left, e_top}); segments.push_back({e_bottom, e_right}); break;
        case 6: case 9: segments.push_back({e_bottom, e_top}); break;
        case 7: case 8: segments.push_back({e_left, e_top}); break;
        case 10: segments.push_back({e_left, e_bottom}); segments.push_back({e_right, e_top}); break;
    }
}

static bool contourPointEq(const ContourPoint& a, const ContourPoint& b, float eps = 1e-3f) {
    return std::abs(a.r - b.r) < eps && std::abs(a.theta - b.theta) < eps;
}

std::vector<std::vector<ContourPoint>> WeldTrajectoryGenerator::segmentsToContours(
    const std::vector<std::pair<ContourPoint, ContourPoint>>& segments) {
    std::vector<std::vector<ContourPoint>> contours;
    if (segments.empty()) return contours;

    std::vector<std::pair<ContourPoint, ContourPoint>> segs = segments;
    std::vector<bool> used(segs.size(), false);

    while (true) {
        int start = -1;
        for (size_t i = 0; i < segs.size(); ++i) {
            if (!used[i]) { start = static_cast<int>(i); break; }
        }
        if (start < 0) break;

        std::vector<ContourPoint> contour;
        contour.push_back(segs[start].first);
        contour.push_back(segs[start].second);
        used[start] = true;

        bool extended = true;
        while (extended) {
            extended = false;
            for (size_t i = 0; i < segs.size() && !extended; ++i) {
                if (used[i]) continue;
                if (contourPointEq(segs[i].first, contour.back())) {
                    contour.push_back(segs[i].second);
                    used[i] = true;
                    extended = true;
                } else if (contourPointEq(segs[i].second, contour.back())) {
                    contour.push_back(segs[i].first);
                    used[i] = true;
                    extended = true;
                } else if (contourPointEq(segs[i].first, contour.front())) {
                    contour.insert(contour.begin(), segs[i].second);
                    used[i] = true;
                    extended = true;
                } else if (contourPointEq(segs[i].second, contour.front())) {
                    contour.insert(contour.begin(), segs[i].first);
                    used[i] = true;
                    extended = true;
                }
            }
        }
        contours.push_back(std::move(contour));
    }
    return contours;
}

std::vector<std::vector<ContourPoint>> WeldTrajectoryGenerator::extractContours(
    const std::array<float, GRID_CELLS_PER_BLADE>& damage_map, float threshold) {
    std::vector<std::pair<ContourPoint, ContourPoint>> segments;
    for (int ri = 0; ri < RADIAL_GRID_N - 1; ++ri) {
        for (int ai = 0; ai < ANGULAR_GRID_N - 1; ++ai) {
            marchingSquaresCell(damage_map, ri, ai, threshold, segments);
        }
    }
    return segmentsToContours(segments);
}

std::vector<RobotWaypoint> WeldTrajectoryGenerator::contourToZigzagFill(
    const std::vector<ContourPoint>& contour, float line_spacing) {
    std::vector<RobotWaypoint> result;
    if (contour.size() < 2) return result;

    float r_min = R_OUTER_MM, r_max = R_INNER_MM;
    float t_min = BLADE_ANGLE_SPAN, t_max = 0.0f;
    for (auto& c : contour) {
        r_min = std::min(r_min, c.r);
        r_max = std::max(r_max, c.r);
        t_min = std::min(t_min, c.theta);
        t_max = std::max(t_max, c.theta);
    }

    auto pointInside = [&](float r, float t) -> bool {
        int crossings = 0;
        size_t n = contour.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            float yi = contour[i].theta, yj = contour[j].theta;
            if (((yi > t) != (yj > t))) {
                float x_intersect = contour[j].r + (contour[i].r - contour[j].r)
                    * (t - yj) / (yi - yj);
                if (r < x_intersect) crossings++;
            }
        }
        return (crossings & 1) != 0;
    };

    bool reverse = false;
    for (float r = r_min; r <= r_max; r += line_spacing) {
        std::vector<float> thetas_in_range;
        for (float t = t_min; t <= t_max; t += line_spacing * 0.5f) {
            if (pointInside(r, t)) thetas_in_range.push_back(t);
        }
        if (thetas_in_range.empty()) continue;
        if (reverse) std::reverse(thetas_in_range.begin(), thetas_in_range.end());
        reverse = !reverse;
        for (float t : thetas_in_range) {
            float theta_global = 0.0f;
            RobotWaypoint wp{};
            Point3D pt = polarToCartesian(r, theta_global, 0);
            wp.x = pt.x; wp.y = pt.y; wp.z = pt.z;
            wp.speed_mm_s = weld_speed_;
            wp.dwell_time_s = 0.0f;
            wp.action_type = 2;
            result.push_back(wp);
        }
    }
    return result;
}

std::vector<RobotWaypoint> WeldTrajectoryGenerator::generate(
    const std::array<float, GRID_CELLS_PER_BLADE>& damage_map,
    uint8_t turbine_id, uint8_t blade_id, float damage_threshold) {
    std::vector<RobotWaypoint> result;
    (void)turbine_id;

    float line_spacing = bead_width_ * (1.0f - overlap_ratio_);

    float thresholds[] = { damage_threshold + 0.5f, damage_threshold + 0.3f, damage_threshold };
    for (float thresh : thresholds) {
        auto contours = extractContours(damage_map, thresh);
        std::sort(contours.begin(), contours.end(),
            [](const std::vector<ContourPoint>& a, const std::vector<ContourPoint>& b) {
                float ra = 0, rb = 0;
                for (auto& p : a) ra += p.r;
                for (auto& p : b) rb += p.r;
                return ra > rb;
            });

        for (auto& contour : contours) {
            if (contour.size() < 3) continue;

            float theta_center = bladeCenterAngle(blade_id);
            for (auto& cp : contour) {
                Point3D p = polarToCartesian(cp.r, theta_center + cp.theta, 0);
                Quaternion q = bladeNormalOrientation(blade_id, cp.r);
                RobotWaypoint wp{};
                wp.x = p.x; wp.y = p.y; wp.z = p.z;
                wp.orientation_w = q.w; wp.orientation_x = q.x;
                wp.orientation_y = q.y; wp.orientation_z = q.z;
                wp.speed_mm_s = weld_speed_;
                wp.dwell_time_s = 0.0f;
                wp.action_type = 2;
                result.push_back(wp);
            }
            if (!result.empty()) result.back().dwell_time_s = 0.1f;

            auto fill = contourToZigzagFill(contour, line_spacing);
            for (auto& wp : fill) {
                float r_mag = std::sqrt(wp.x*wp.x + wp.y*wp.y);
                float local_t = std::atan2(wp.y, wp.x) - theta_center;
                while (local_t < 0) local_t += TWO_PI;
                while (local_t >= TWO_PI) local_t -= TWO_PI;
                float local_t_clamped = std::clamp(local_t, 0.0f, BLADE_ANGLE_SPAN);
                Point3D p = polarToCartesian(r_mag, theta_center + local_t_clamped, 0);
                Quaternion q = bladeNormalOrientation(blade_id, r_mag);
                wp.x = p.x; wp.y = p.y; wp.z = p.z;
                wp.orientation_w = q.w; wp.orientation_x = q.x;
                wp.orientation_y = q.y; wp.orientation_z = q.z;
                result.push_back(wp);
            }
        }
    }
    return result;
}

PolishTrajectoryGenerator::PolishTrajectoryGenerator()
    : grid_step_(0.5f), polish_speed_(20.0f), normal_force_(5.0f) {}

void PolishTrajectoryGenerator::setGridStep(float step_mm) { grid_step_ = step_mm; }
void PolishTrajectoryGenerator::setPolishSpeed(float speed_mm_s) { polish_speed_ = speed_mm_s; }
void PolishTrajectoryGenerator::setForceNormal(float force_n) { normal_force_ = force_n; }

float PolishTrajectoryGenerator::twistAngle(float r_norm) {
    return TWIST_RAD_MAX * std::clamp(r_norm, 0.0f, 1.0f);
}

Point3D PolishTrajectoryGenerator::bladeSurfacePoint(uint8_t blade_id, float r,
    float theta_local, float z_offset) {
    float theta_center = bladeCenterAngle(blade_id);
    float r_norm = (r - R_INNER_MM) / (R_OUTER_MM - R_INNER_MM);
    float twist = twistAngle(r_norm);
    float theta = theta_center + theta_local;
    Point3D p = polarToCartesian(r, theta, z_offset);
    p.z += r * std::sin(twist) * (theta_local - BLADE_ANGLE_SPAN * 0.5f) / BLADE_ANGLE_SPAN;
    return p;
}

Quaternion PolishTrajectoryGenerator::bladeNormalOrientation(uint8_t blade_id,
    float r, float theta_local) {
    float theta_center = bladeCenterAngle(blade_id);
    float r_norm = (r - R_INNER_MM) / (R_OUTER_MM - R_INNER_MM);
    float twist = twistAngle(r_norm);
    float blade_surface = theta_center + BLADE_ANGLE_SPAN * 0.5f;

    float nx = -std::sin(blade_surface) * std::cos(twist);
    float ny =  std::cos(blade_surface) * std::cos(twist);
    float nz =  std::sin(twist);

    float nlen = std::sqrt(nx*nx + ny*ny + nz*nz);
    if (nlen > 1e-6f) { nx /= nlen; ny /= nlen; nz /= nlen; }

    float upx = 0, upy = 0, upz = 1;
    float tx = upy * nz - upz * ny;
    float ty = upz * nx - upx * nz;
    float tz = upx * ny - upy * nx;
    float tlen = std::sqrt(tx*tx + ty*ty + tz*tz);
    if (tlen < 1e-6f) { tx = 1; ty = 0; tz = 0; }
    else { tx /= tlen; ty /= tlen; tz /= tlen; }

    float bx = ny * tz - nz * ty;
    float by = nz * tx - nx * tz;
    float bz = nx * ty - ny * tx;

    float m00 = tx, m01 = bx, m02 = nx;
    float m10 = ty, m11 = by, m12 = ny;
    float m20 = tz, m21 = bz, m22 = nz;

    float tr = m00 + m11 + m22;
    Quaternion q;
    if (tr > 0) {
        float s = std::sqrt(tr + 1.0f) * 2.0f;
        q.w = 0.25f * s;
        q.x = (m21 - m12) / s;
        q.y = (m02 - m20) / s;
        q.z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        float s = std::sqrt(1.0f + m00 - m11 - m22) * 2;
        q.w = (m21 - m12) / s;
        q.x = 0.25f * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
    } else if (m11 > m22) {
        float s = std::sqrt(1.0f + m11 - m00 - m22) * 2;
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = 0.25f * s;
        q.z = (m12 + m21) / s;
    } else {
        float s = std::sqrt(1.0f + m22 - m00 - m11) * 2;
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25f * s;
    }
    q.normalize();
    (void)theta_local;
    return q;
}

std::vector<RobotWaypoint> PolishTrajectoryGenerator::generate(
    const std::array<float, GRID_CELLS_PER_BLADE>& damage_map,
    uint8_t turbine_id, uint8_t blade_id, float damage_threshold) {
    std::vector<RobotWaypoint> result;
    (void)turbine_id;

    float r_range = R_OUTER_MM - R_INNER_MM;
    int r_steps = std::max(1, static_cast<int>(std::round(r_range / grid_step_)));
    int t_steps = std::max(1, static_cast<int>(std::round(
        BLADE_ANGLE_SPAN * 0.5f * (R_INNER_MM + R_OUTER_MM) / grid_step_)));

    for (int ri = 0; ri <= r_steps; ++ri) {
        float r = R_INNER_MM + (static_cast<float>(ri) / r_steps) * r_range;
        bool reverse = (ri & 1) != 0;
        int t_start = reverse ? t_steps : 0;
        int t_end = reverse ? -1 : t_steps + 1;
        int t_inc = reverse ? -1 : 1;

        for (int ti = t_start; ti != t_end; ti += t_inc) {
            float theta_local = (static_cast<float>(ti) / t_steps) * BLADE_ANGLE_SPAN;
            uint8_t grid_ri = std::min<uint8_t>(
                static_cast<uint8_t>(ri * RADIAL_GRID_N / std::max<int>(r_steps,1)), RADIAL_GRID_N-1);
            uint8_t grid_ai = std::min<uint8_t>(
                static_cast<uint8_t>(ti * ANGULAR_GRID_N / std::max<int>(t_steps,1)), ANGULAR_GRID_N-1);
            float dmg = damage_map[bladeGridIndex(grid_ri, grid_ai)];
            if (dmg < damage_threshold && ri > 0 && ri < r_steps - 1) {
                if (!result.empty()) continue;
            }
            float z_off = -normal_force_ * 0.02f;
            Point3D p = bladeSurfacePoint(blade_id, r, theta_local, z_off);
            Quaternion q = bladeNormalOrientation(blade_id, r, theta_local);
            RobotWaypoint wp{};
            wp.x = p.x; wp.y = p.y; wp.z = p.z;
            wp.orientation_w = q.w; wp.orientation_x = q.x;
            wp.orientation_y = q.y; wp.orientation_z = q.z;
            wp.speed_mm_s = polish_speed_;
            wp.dwell_time_s = 0.0f;
            wp.action_type = 1;
            result.push_back(wp);
        }
    }
    return result;
}

RobotPlannerFacade::RobotPlannerFacade() {
    rrt_planner_.setWorkspace(R_INNER_MM, R_OUTER_MM, BLADE_THICKNESS_MM);
    rrt_planner_.setSafetyDistance(SAFETY_DISTANCE_MM);
    weld_gen_.setBeadWidth(6.0f);
    weld_gen_.setOverlapRatio(0.3f);
    polish_gen_.setGridStep(0.5f);
}

void RobotPlannerFacade::setConfig(const Config::RobotConfig& cfg) {
    config_ = cfg;
    weld_gen_.setBeadWidth(cfg.weld_bead_width_mm);
    weld_gen_.setWeldSpeed(cfg.weld_speed_mm_s);
    polish_gen_.setGridStep(cfg.polish_step_mm);
    polish_gen_.setPolishSpeed(cfg.polish_speed_mm_s);
    rrt_planner_.setWorkspace(cfg.turbine_inner_radius_mm,
                               cfg.turbine_outer_radius_mm,
                               BLADE_THICKNESS_MM);
    rrt_planner_.setSafetyDistance(cfg.safety_clearance_mm);
}

bool RobotPlannerFacade::autoTriggerCheck(
    const std::vector<LifeAssessment>& life_assessment_data) {
    float threshold_pct = config_.auto_trigger_life_pct > 0 ? config_.auto_trigger_life_pct : 10.0f;
    for (auto& life : life_assessment_data) {
        float expected_life = 100000.0f;
        float life_pct = 100.0f * (1.0f - std::clamp(life.cumulative_damage, 0.0f, 1.0f));
        if (life_pct < threshold_pct) return true;
        if (life.remaining_life_hours > 0 && expected_life > 0) {
            float pct2 = 100.0f * life.remaining_life_hours / expected_life;
            if (pct2 < threshold_pct) return true;
        }
    }
    return false;
}

RobotPose RobotPlannerFacade::computeBladeInspectionPose(uint8_t blade_id, uint8_t waypoint_idx) {
    float theta_center = bladeCenterAngle(blade_id);
    uint8_t total = config_.path_waypoint_count > 0 ? config_.path_waypoint_count : 64;
    float t = static_cast<float>(waypoint_idx % 16) / 15.0f;
    float r = R_INNER_MM + t * (R_OUTER_MM - R_INNER_MM);
    int ring = waypoint_idx / 16;
    float angle_offset = (ring % 2 == 0 ? -1 : 1) * BLADE_ANGLE_SPAN * 0.3f;
    float local_t = BLADE_ANGLE_SPAN * 0.5f + angle_offset * (0.3f + 0.4f * ((waypoint_idx % 8) / 7.0f));
    float z = (ring % 4 - 1.5f) * (BLADE_THICKNESS_MM * 0.25f);
    float standoff = config_.inspection_grid_step_mm > 0 ? config_.inspection_grid_step_mm : 50.0f;

    float theta = theta_center + local_t;
    Point3D surface_pt = polarToCartesian(r, theta, z);
    float nx = -std::cos(theta);
    float ny = -std::sin(theta);
    float nz = 0.0f;
    Point3D camera_pt(
        surface_pt.x + nx * standoff,
        surface_pt.y + ny * standoff,
        surface_pt.z + nz * standoff
    );

    float yaw = theta + PI;
    float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
    Quaternion orient(cy, 0, 0, sy);
    return RobotPose(camera_pt, orient);
}

RobotRepairTask RobotPlannerFacade::planFullInspection(uint8_t turbine_id) {
    RobotRepairTask task{};
    task.timestamp = currentTimestampMs();
    task.turbine_id = turbine_id;
    task.robot_status = RobotStatus::PLANNING;
    task.repair_mode = RepairMode::INSPECTION_ONLY;
    for (uint8_t b = 1; b <= BLADES_PER_TURBINE; ++b) task.target_blade_ids.push_back(b);
    task.current_waypoint_idx = 0;

    uint8_t wp_per_blade = 4;
    uint8_t total_wp = BLADES_PER_TURBINE * wp_per_blade;
    task.inspection_path.reserve(total_wp);

    RobotPose home_pos(Point3D(0, 0, BLADE_THICKNESS_MM * 0.6f), Quaternion(1,0,0,0));
    RobotPose prev_pose = home_pos;

    for (uint8_t blade = 1; blade <= BLADES_PER_TURBINE; ++blade) {
        for (uint8_t wpi = 0; wpi < wp_per_blade; ++wpi) {
            uint8_t idx = (wpi * 16 / wp_per_blade);
            RobotPose pose = computeBladeInspectionPose(blade, idx);
            auto blade_constraints = [](const Point3D&) { return true; };
            auto raw_path = rrt_planner_.plan(prev_pose, pose, blade_constraints);
            auto smooth_path = rrt_planner_.smooth(raw_path, 1.0f);
            for (auto& wp : smooth_path) { wp.action_type = 0; task.inspection_path.push_back(wp); }
            RobotWaypoint stop{};
            stop.x = pose.position.x; stop.y = pose.position.y; stop.z = pose.position.z;
            stop.orientation_w = pose.orientation.w; stop.orientation_x = pose.orientation.x;
            stop.orientation_y = pose.orientation.y; stop.orientation_z = pose.orientation.z;
            stop.speed_mm_s = 0; stop.dwell_time_s = 0.5f; stop.action_type = 0;
            task.inspection_path.push_back(stop);
            prev_pose = pose;
        }
    }
    auto return_path = rrt_planner_.plan(prev_pose, home_pos, [](const Point3D&){return true;});
    auto return_smooth = rrt_planner_.smooth(return_path, 1.0f);
    for (auto& wp : return_smooth) { wp.action_type = 0; task.inspection_path.push_back(wp); }

    auto dmg_map = heatMap_.getBladeDamageMap(turbine_id, 1);
    task.blade_damage_map.insert(task.blade_damage_map.end(), dmg_map.begin(), dmg_map.end());

    task.estimated_duration_ms = static_cast<uint64_t>(estimateTaskDuration(task));
    task.total_repair_area_cm2 = 0;
    task.total_weld_volume_cm3 = 0;
    task.repair_sequence = "INSPECTION:" + std::to_string(BLADES_PER_TURBINE) + "blades";
    task.robot_pos[0] = home_pos.position.x;
    task.robot_pos[1] = home_pos.position.y;
    task.robot_pos[2] = home_pos.position.z;
    task.robot_status = RobotStatus::IDLE;
    return task;
}

std::vector<uint8_t> RobotPlannerFacade::selectBladesByDamage(uint8_t turbine_id, float min_damage) {
    std::vector<uint8_t> result;
    for (uint8_t b = 1; b <= BLADES_PER_TURBINE; ++b) {
        auto mp = heatMap_.getBladeDamageMap(turbine_id, b);
        float avg = std::accumulate(mp.begin(), mp.end(), 0.0f) / mp.size();
        float maxd = *std::max_element(mp.begin(), mp.end());
        if (maxd >= min_damage || avg >= min_damage * 0.5f) {
            result.push_back(b);
        }
    }
    return result;
}

float RobotPlannerFacade::computeRepairArea(
    const std::array<float, GRID_CELLS_PER_BLADE>& map, float threshold) {
    float area = 0.0f;
    float dr = (R_OUTER_MM - R_INNER_MM) / RADIAL_GRID_N;
    for (uint8_t ri = 0; ri < RADIAL_GRID_N; ++ri) {
        float r = radialIndexToRadius(ri);
        float dtheta = BLADE_ANGLE_SPAN / ANGULAR_GRID_N;
        float cell_area = r * dtheta * dr;
        for (uint8_t ai = 0; ai < ANGULAR_GRID_N; ++ai) {
            if (map[bladeGridIndex(ri, ai)] >= threshold) {
                area += cell_area;
            }
        }
    }
    return area / 100.0f;
}

float RobotPlannerFacade::computeWeldVolume(float area_cm2, float avg_depth_mm) {
    return area_cm2 * (avg_depth_mm / 10.0f) * 0.7f;
}

float RobotPlannerFacade::estimateTaskDuration(const RobotRepairTask& task) {
    float total_ms = 0.0f;
    auto calc_segment = [&](const std::vector<RobotWaypoint>& path, float default_speed) {
        if (path.empty()) return 0.0f;
        float t = 0;
        for (size_t i = 0; i < path.size(); ++i) {
            t += path[i].dwell_time_s * 1000.0f;
            if (i > 0) {
                float dx = path[i].x - path[i-1].x;
                float dy = path[i].y - path[i-1].y;
                float dz = path[i].z - path[i-1].z;
                float d = std::sqrt(dx*dx + dy*dy + dz*dz);
                float spd = path[i].speed_mm_s > 0 ? path[i].speed_mm_s : default_speed;
                t += (d / spd) * 1000.0f;
            }
        }
        return t;
    };
    total_ms += calc_segment(task.inspection_path, config_.robot_speed_mm_s);
    total_ms += calc_segment(task.polish_trajectory, config_.polish_speed_mm_s);
    total_ms += calc_segment(task.weld_trajectory, config_.weld_speed_mm_s);
    return total_ms;
}

RobotRepairTask RobotPlannerFacade::planRepair(uint8_t turbine_id,
    const std::vector<uint8_t>& blade_ids, RepairMode repair_mode) {
    RobotRepairTask task{};
    task.timestamp = currentTimestampMs();
    task.turbine_id = turbine_id;
    task.robot_status = RobotStatus::PLANNING;
    task.repair_mode = repair_mode;
    task.target_blade_ids = blade_ids.empty() ? selectBladesByDamage(turbine_id, 0.1f) : blade_ids;
    task.current_waypoint_idx = 0;

    RobotPose home(Point3D(0, 0, BLADE_THICKNESS_MM * 0.6f), Quaternion(1,0,0,0));
    RobotPose prev = home;
    auto default_constraints = [](const Point3D&) { return true; };

    for (uint8_t blade : task.target_blade_ids) {
        auto dmg = heatMap_.getBladeDamageMap(turbine_id, blade);

        for (uint8_t wpi = 0; wpi < 8; ++wpi) {
            RobotPose insp = computeBladeInspectionPose(blade, wpi * 2);
            auto raw = rrt_planner_.plan(prev, insp, default_constraints);
            auto sm = rrt_planner_.smooth(raw, 1.0f);
            for (auto& wp : sm) { wp.action_type = 0; task.inspection_path.push_back(wp); }
            RobotWaypoint s{};
            s.x = insp.position.x; s.y = insp.position.y; s.z = insp.position.z;
            s.orientation_w = insp.orientation.w; s.orientation_x = insp.orientation.x;
            s.orientation_y = insp.orientation.y; s.orientation_z = insp.orientation.z;
            s.dwell_time_s = 0.5f; s.action_type = 0;
            task.inspection_path.push_back(s);
            prev = insp;
        }
        task.total_repair_area_cm2 += computeRepairArea(dmg, 0.1f);

        if (repair_mode == RepairMode::POLISH || repair_mode == RepairMode::POLISH_AND_WELD) {
            auto poly = polish_gen_.generate(dmg, turbine_id, blade, 0.1f);
            if (!poly.empty()) {
                RobotPose first_poly_pose(Point3D(poly.front().x, poly.front().y, poly.front().z + 30),
                    Quaternion(poly.front().orientation_w, poly.front().orientation_x,
                               poly.front().orientation_y, poly.front().orientation_z));
                auto approach = rrt_planner_.plan(prev, first_poly_pose, default_constraints);
                auto app_s = rrt_planner_.smooth(approach, 1.0f);
                for (auto& wp : app_s) { wp.action_type = 1; task.polish_trajectory.push_back(wp); }
                for (auto& wp : poly) task.polish_trajectory.push_back(wp);
                prev = RobotPose(Point3D(poly.back().x, poly.back().y, poly.back().z),
                    Quaternion(poly.back().orientation_w, poly.back().orientation_x,
                               poly.back().orientation_y, poly.back().orientation_z));
            }
        }

        if (repair_mode == RepairMode::WELD || repair_mode == RepairMode::POLISH_AND_WELD) {
            auto wtp = weld_gen_.generate(dmg, turbine_id, blade, 0.3f);
            if (!wtp.empty()) {
                RobotPose first_weld_pose(Point3D(wtp.front().x, wtp.front().y, wtp.front().z + 20),
                    Quaternion(wtp.front().orientation_w, wtp.front().orientation_x,
                               wtp.front().orientation_y, wtp.front().orientation_z));
                auto approach = rrt_planner_.plan(prev, first_weld_pose, default_constraints);
                auto app_s = rrt_planner_.smooth(approach, 1.0f);
                for (auto& wp : app_s) { wp.action_type = 2; task.weld_trajectory.push_back(wp); }
                for (auto& wp : wtp) task.weld_trajectory.push_back(wp);
                prev = RobotPose(Point3D(wtp.back().x, wtp.back().y, wtp.back().z),
                    Quaternion(wtp.back().orientation_w, wtp.back().orientation_x,
                               wtp.back().orientation_y, wtp.back().orientation_z));
            }
        }

        auto blade_map = heatMap_.getBladeDamageMap(turbine_id, blade);
        task.blade_damage_map.insert(task.blade_damage_map.end(), blade_map.begin(), blade_map.end());
    }

    auto ret_path = rrt_planner_.plan(prev, home, default_constraints);
    auto ret_sm = rrt_planner_.smooth(ret_path, 1.0f);
    for (auto& wp : ret_sm) task.inspection_path.push_back(wp);

    task.total_weld_volume_cm3 = computeWeldVolume(task.total_repair_area_cm2, WELD_DEPTH_MM);
    task.estimated_duration_ms = static_cast<uint64_t>(estimateTaskDuration(task));

    std::string seq;
    if (repair_mode == RepairMode::INSPECTION_ONLY) seq = "INSPECT";
    else if (repair_mode == RepairMode::POLISH) seq = "INSPECT->POLISH->RETURN";
    else if (repair_mode == RepairMode::WELD) seq = "INSPECT->WELD->RETURN";
    else seq = "INSPECT->POLISH->WELD->RETURN";
    seq += " blades=" + std::to_string(task.target_blade_ids.size());
    task.repair_sequence = seq;

    task.robot_pos[0] = home.position.x;
    task.robot_pos[1] = home.position.y;
    task.robot_pos[2] = home.position.z;
    task.robot_status = RobotStatus::IDLE;
    return task;
}

RobotWaypoint RobotPlannerFacade::simulateExecution(RobotRepairTask& task, uint32_t time_ms) {
    static std::map<uint64_t, uint32_t> accum_time;
    uint64_t task_id = task.timestamp;
    accum_time[task_id] += time_ms;
    uint32_t elapsed = accum_time[task_id];

    const std::vector<RobotWaypoint>* current_path = nullptr;
    RobotStatus active_status = task.robot_status;
    switch (active_status) {
        case RobotStatus::INSPECTING:
        case RobotStatus::DEPLOYING:
        case RobotStatus::RETURNING:
            current_path = &task.inspection_path; break;
        case RobotStatus::POLISHING:
            current_path = &task.polish_trajectory; break;
        case RobotStatus::WELDING:
            current_path = &task.weld_trajectory; break;
        default:
            current_path = &task.inspection_path; break;
    }

    if (!current_path || current_path->empty()) {
        RobotWaypoint wp{};
        wp.x = task.robot_pos[0]; wp.y = task.robot_pos[1]; wp.z = task.robot_pos[2];
        wp.orientation_w = 1;
        return wp;
    }

    uint32_t idx = task.current_waypoint_idx;
    if (idx >= current_path->size()) idx = static_cast<uint32_t>(current_path->size() - 1);
    const RobotWaypoint& target = (*current_path)[idx];

    RobotWaypoint result = target;
    float total_d = 0;
    if (idx > 0) {
        const RobotWaypoint& prev = (*current_path)[idx - 1];
        float dx = target.x - prev.x, dy = target.y - prev.y, dz = target.z - prev.z;
        total_d = std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    float speed = target.speed_mm_s > 0 ? target.speed_mm_s : config_.robot_speed_mm_s;
    float travel_t_ms = (speed > 0 && total_d > 0) ? (total_d / speed) * 1000.0f : 10.0f;
    float dwell_ms = target.dwell_time_s * 1000.0f;
    float seg_total = travel_t_ms + dwell_ms;

    uint32_t seg_elapsed = elapsed % std::max<uint32_t>(1, static_cast<uint32_t>(seg_total));
    float t = seg_total > 0 ? (seg_elapsed / seg_total) : 1.0f;

    if (t < 1.0f && idx > 0) {
        const RobotWaypoint& prev = (*current_path)[idx - 1];
        float move_t = std::min(1.0f, t * seg_total / std::max(1e-6f, travel_t_ms));
        result.x = prev.x + (target.x - prev.x) * move_t;
        result.y = prev.y + (target.y - prev.y) * move_t;
        result.z = prev.z + (target.z - prev.z) * move_t;
        result.orientation_w = prev.orientation_w + (target.orientation_w - prev.orientation_w) * move_t;
        result.orientation_x = prev.orientation_x + (target.orientation_x - prev.orientation_x) * move_t;
        result.orientation_y = prev.orientation_y + (target.orientation_y - prev.orientation_y) * move_t;
        result.orientation_z = prev.orientation_z + (target.orientation_z - prev.orientation_z) * move_t;
    }
    result.orientation.normalize ? result.orientation : result.orientation;
    {
        float qlen = std::sqrt(result.orientation_w * result.orientation_w
            + result.orientation_x * result.orientation_x
            + result.orientation_y * result.orientation_y
            + result.orientation_z * result.orientation_z);
        if (qlen > 1e-6f) {
            result.orientation_w /= qlen;
            result.orientation_x /= qlen;
            result.orientation_y /= qlen;
            result.orientation_z /= qlen;
        }
    }

    float seg_steps = std::max<uint32_t>(1, static_cast<uint32_t>(std::ceil(seg_total / 20.0f)));
    uint32_t advance_every = std::max<uint32_t>(1, static_cast<uint32_t>(seg_steps));
    if (elapsed / advance_every > (elapsed - time_ms) / advance_every || seg_elapsed >= seg_total - 1) {
        if (seg_elapsed >= seg_total - 1 || t >= 0.999f) {
            if (task.current_waypoint_idx < static_cast<uint32_t>(current_path->size() - 1)) {
                task.current_waypoint_idx++;
            } else {
                switch (active_status) {
                    case RobotStatus::DEPLOYING:
                        task.robot_status = RobotStatus::INSPECTING;
                        task.current_waypoint_idx = 0;
                        accum_time[task_id] = 0;
                        break;
                    case RobotStatus::INSPECTING:
                        if (!task.polish_trajectory.empty() && (task.repair_mode == RepairMode::POLISH
                            || task.repair_mode == RepairMode::POLISH_AND_WELD)) {
                            task.robot_status = RobotStatus::POLISHING;
                        } else if (!task.weld_trajectory.empty() && task.repair_mode == RepairMode::WELD) {
                            task.robot_status = RobotStatus::WELDING;
                        } else {
                            task.robot_status = RobotStatus::RETURNING;
                        }
                        task.current_waypoint_idx = 0;
                        accum_time[task_id] = 0;
                        break;
                    case RobotStatus::POLISHING:
                        if (!task.weld_trajectory.empty() && task.repair_mode == RepairMode::POLISH_AND_WELD) {
                            task.robot_status = RobotStatus::WELDING;
                        } else {
                            task.robot_status = RobotStatus::RETURNING;
                        }
                        task.current_waypoint_idx = 0;
                        accum_time[task_id] = 0;
                        break;
                    case RobotStatus::WELDING:
                        task.robot_status = RobotStatus::RETURNING;
                        task.current_waypoint_idx = 0;
                        accum_time[task_id] = 0;
                        break;
                    case RobotStatus::RETURNING:
                        task.robot_status = RobotStatus::COMPLETED;
                        accum_time.erase(task_id);
                        break;
                    default:
                        break;
                }
            }
        }
    }

    task.robot_pos[0] = result.x;
    task.robot_pos[1] = result.y;
    task.robot_pos[2] = result.z;
    return result;
}

}
