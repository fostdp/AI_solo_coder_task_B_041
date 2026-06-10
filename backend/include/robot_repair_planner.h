#pragma once

#include "robot_planner.h"

namespace turbine_monitor {

using Point3D              = robot_plan::Point3D;
using Quaternion           = robot_plan::Quaternion;
using RobotPose            = robot_plan::RobotPose;
using ContourPoint         = robot_plan::ContourPoint;
using DynamicPositioningConfig = robot_plan::DynamicPositioningConfig;
using DisturbanceObserver  = robot_plan::DisturbanceObserver;
using DynamicPositioner    = robot_plan::DynamicPositioner;
using DamageHeatMap        = robot_plan::DamageHeatMap;
using RRTStarPathPlanner   = robot_plan::RRTStarPathPlanner;
using WeldTrajectoryGenerator   = robot_plan::WeldTrajectoryGenerator;
using PolishTrajectoryGenerator = robot_plan::PolishTrajectoryGenerator;
using RobotPlannerFacade   = robot_plan::RobotPlannerFacade;

static constexpr auto TURBINE_TOTAL       = robot_plan::TURBINE_TOTAL;
static constexpr auto BLADES_PER_TURBINE  = robot_plan::BLADES_PER_TURBINE;
static constexpr auto RADIAL_GRID_N       = robot_plan::RADIAL_GRID_N;
static constexpr auto ANGULAR_GRID_N      = robot_plan::ANGULAR_GRID_N;
static constexpr auto GRID_CELLS_PER_BLADE = robot_plan::GRID_CELLS_PER_BLADE;
static constexpr auto TOTAL_DAMAGE_GRIDS  = robot_plan::TOTAL_DAMAGE_GRIDS;

}
