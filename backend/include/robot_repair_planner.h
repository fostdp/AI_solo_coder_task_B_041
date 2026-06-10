#pragma once

#include "data_structures.h"
#include "config.h"
#include <vector>
#include <array>
#include <memory>
#include <random>
#include <string>
#include <cmath>
#include <functional>

namespace turbine_monitor {

constexpr uint8_t  TURBINE_TOTAL       = TURBINE_COUNT;
constexpr uint8_t  BLADES_PER_TURBINE  = BLADE_COUNT;
constexpr uint8_t  RADIAL_GRID_N       = 9;
constexpr uint8_t  ANGULAR_GRID_N      = 7;
constexpr uint16_t GRID_CELLS_PER_BLADE = RADIAL_GRID_N * ANGULAR_GRID_N;
constexpr uint32_t TOTAL_DAMAGE_GRIDS  = TURBINE_TOTAL * BLADES_PER_TURBINE * GRID_CELLS_PER_BLADE;

constexpr float R_INNER_MM  = 2500.0f;
constexpr float R_OUTER_MM  = 5000.0f;
constexpr float BLADE_THICKNESS_MM = 1200.0f;
constexpr float TWIST_RAD_MAX = 0.5f;
constexpr float SAFETY_DISTANCE_MM = 50.0f;
constexpr float RRT_REWIRE_RADIUS_MM = 50.0f;

struct Point3D {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    Point3D() = default;
    Point3D(float xv, float yv, float zv) : x(xv), y(yv), z(zv) {}
    float distance(const Point3D& other) const {
        float dx = x - other.x, dy = y - other.y, dz = z - other.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    Point3D operator+(const Point3D& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Point3D operator-(const Point3D& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Point3D operator*(float s) const { return {x*s, y*s, z*s}; }
};

struct Quaternion {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
    Quaternion() = default;
    Quaternion(float wv, float xv, float yv, float zv) : w(wv), x(xv), y(yv), z(zv) {}
    void normalize() {
        float n = std::sqrt(w*w + x*x + y*y + z*z);
        if (n > 1e-6f) { w/=n; x/=n; y/=n; z/=n; }
    }
    static Quaternion fromAngleAxis(float angle, float ax, float ay, float az) {
        float ha = angle * 0.5f;
        float s = std::sin(ha);
        return Quaternion(std::cos(ha), ax*s, ay*s, az*s);
    }
};

struct RobotPose {
    Point3D    position;
    Quaternion orientation;
    RobotPose() = default;
    RobotPose(const Point3D& p, const Quaternion& q) : position(p), orientation(q) {}
};

struct ContourPoint {
    float r = 0.0f;
    float theta = 0.0f;
    ContourPoint() = default;
    ContourPoint(float rv, float tv) : r(rv), theta(tv) {}
};

class DamageHeatMap {
public:
    DamageHeatMap();
    ~DamageHeatMap() = default;

    void update(uint8_t turbine_id, uint8_t blade_id,
                float cumulative_damage, float cavitation_intensity);

    std::array<float, GRID_CELLS_PER_BLADE> getBladeDamageMap(
        uint8_t turbine_id, uint8_t blade_id) const;

    float getCellDamage(uint8_t turbine_id, uint8_t blade_id,
                        uint8_t radial_idx, uint8_t angular_idx) const;

    void setCellDamage(uint8_t turbine_id, uint8_t blade_id,
                       uint8_t radial_idx, uint8_t angular_idx, float value);

    void computeDamageDistribution(uint8_t turbine_id, uint8_t blade_id,
                                   float cumulative_damage, float cavitation_intensity);

private:
    size_t cellIndex(uint8_t turbine_id, uint8_t blade_id,
                     uint8_t radial_idx, uint8_t angular_idx) const;

    std::vector<float> damageGrid_;
};

class RRTStarPathPlanner {
public:
    RRTStarPathPlanner();
    ~RRTStarPathPlanner() = default;

    void setWorkspace(float inner_radius_mm, float outer_radius_mm, float thickness_mm);
    void setSafetyDistance(float mm);
    void setMaxIterations(uint32_t iter);
    void setStepSize(float mm);

    std::vector<RobotWaypoint> plan(
        const RobotPose& start, const RobotPose& goal,
        const std::function<bool(const Point3D&)>& blade_surface_constraints);

    std::vector<RobotWaypoint> smooth(const std::vector<RobotWaypoint>& path,
                                      float control_interval_mm = 1.0f);

private:
    struct RRTNode {
        RobotPose pose;
        int32_t   parent = -1;
        float     cost = 0.0f;
        uint32_t  id = 0;
    };

    Point3D sampleFree();
    int32_t  nearest(const Point3D& p);
    std::vector<int32_t> near(const Point32& p, float radius);
    RobotPose steer(const RobotPose& from, const Point3D& to, float step);
    bool    isInWorkspace(const Point3D& p) const;
    bool    checkCollisionLine(const Point3D& a, const Point3D& b,
                               const std::function<bool(const Point3D&)>& constraints);
    float   lineCost(const RobotPose& a, const RobotPose& b);
    RobotWaypoint poseToWaypoint(const RobotPose& pose, float speed_mm_s);
    std::vector<Point3D> bspline3(const std::vector<Point3D>& control_points, int samples);

    float    r_inner_;
    float    r_outer_;
    float    thickness_;
    float    safety_dist_;
    uint32_t max_iter_;
    float    step_size_;
    float    rewire_radius_;

    std::vector<RRTNode> nodes_;
    std::mt19937 rng_;
};

class WeldTrajectoryGenerator {
public:
    WeldTrajectoryGenerator();
    ~WeldTrajectoryGenerator() = default;

    void setBeadWidth(float width_mm);
    void setOverlapRatio(float ratio);
    void setWeldSpeed(float speed_mm_s);

    std::vector<RobotWaypoint> generate(
        const std::array<float, GRID_CELLS_PER_BLADE>& damage_map,
        uint8_t turbine_id, uint8_t blade_id,
        float damage_threshold = 0.3f);

    std::vector<std::vector<ContourPoint>> extractContours(
        const std::array<float, GRID_CELLS_PER_BLADE>& damage_map,
        float threshold);

private:
    Point3D bladeSurfacePoint(uint8_t blade_id, float r, float z_offset);
    Quaternion bladeNormalOrientation(uint8_t blade_id, float r);
    void marchingSquaresCell(
        const std::array<float, GRID_CELLS_PER_BLADE>& grid,
        int ri, int ai, float threshold,
        std::vector<std::pair<ContourPoint, ContourPoint>>& segments);
    ContourPoint interpolate(float v0, float v1, const ContourPoint& p0,
                             const ContourPoint& p1, float threshold);
    std::vector<std::vector<ContourPoint>> segmentsToContours(
        const std::vector<std::pair<ContourPoint, ContourPoint>>& segments);
    std::vector<RobotWaypoint> contourToZigzagFill(
        const std::vector<ContourPoint>& contour, float line_spacing);

    float bead_width_;
    float overlap_ratio_;
    float weld_speed_;
};

class PolishTrajectoryGenerator {
public:
    PolishTrajectoryGenerator();
    ~PolishTrajectoryGenerator() = default;

    void setGridStep(float step_mm);
    void setPolishSpeed(float speed_mm_s);
    void setForceNormal(float force_n);

    std::vector<RobotWaypoint> generate(
        const std::array<float, GRID_CELLS_PER_BLADE>& damage_map,
        uint8_t turbine_id, uint8_t blade_id,
        float damage_threshold = 0.1f);

private:
    Point3D bladeSurfacePoint(uint8_t blade_id, float r, float theta_local, float z_offset);
    Quaternion bladeNormalOrientation(uint8_t blade_id, float r, float theta_local);
    float twistAngle(float r_norm);

    float grid_step_;
    float polish_speed_;
    float normal_force_;
};

class RobotPlannerFacade {
public:
    RobotPlannerFacade();
    ~RobotPlannerFacade() = default;

    void setConfig(const Config::RobotConfig& cfg);
    bool autoTriggerCheck(const std::vector<LifeAssessment>& life_assessment_data);

    RobotRepairTask planFullInspection(uint8_t turbine_id);

    RobotRepairTask planRepair(uint8_t turbine_id,
                               const std::vector<uint8_t>& blade_ids,
                               RepairMode repair_mode);

    RobotWaypoint simulateExecution(RobotRepairTask& task, uint32_t time_ms);

    DamageHeatMap& heatMap() { return heatMap_; }
    const DamageHeatMap& heatMap() const { return heatMap_; }

private:
    RobotPose computeBladeInspectionPose(uint8_t blade_id, uint8_t waypoint_idx);
    std::vector<uint8_t> selectBladesByDamage(uint8_t turbine_id, float min_damage);
    float estimateTaskDuration(const RobotRepairTask& task);
    float computeRepairArea(const std::array<float, GRID_CELLS_PER_BLADE>& map, float threshold);
    float computeWeldVolume(float area_cm2, float avg_depth_mm);

    Config::RobotConfig config_;
    DamageHeatMap       heatMap_;
    RRTStarPathPlanner  rrt_planner_;
    WeldTrajectoryGenerator   weld_gen_;
    PolishTrajectoryGenerator polish_gen_;
};

}
