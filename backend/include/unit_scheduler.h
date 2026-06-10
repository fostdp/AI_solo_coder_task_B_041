#pragma once

#include "plant_scheduler.h"

namespace turbine_monitor {

using UnitEfficiencyCurve  = plant_sched::UnitEfficiencyCurve;
using MILPSolution         = plant_sched::MILPSolution;
using MILPFormulation      = plant_sched::MILPFormulation;
using PlantScheduler       = plant_sched::PlantScheduler;
using SchedulerConfig      = plant_sched::SchedulerConfig;

static constexpr auto SCHED_UNITS   = plant_sched::SCHED_UNITS;
static constexpr auto SCHED_HOURS   = plant_sched::SCHED_HOURS;
static constexpr auto HILL_LOAD_PTS = plant_sched::HILL_LOAD_PTS;
static constexpr auto HILL_HEAD_PTS = plant_sched::HILL_HEAD_PTS;
static constexpr auto MIN_STABLE_HOURS = plant_sched::MIN_STABLE_HOURS;

}
