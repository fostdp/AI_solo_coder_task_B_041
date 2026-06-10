#pragma once

#include "cavitation_controller.h"

namespace turbine_monitor {

using MPCState            = cav_ctrl::MPCState;
using MPCInput            = cav_ctrl::MPCInput;
using MPCWeights          = cav_ctrl::MPCWeights;
using MPCConstraints      = cav_ctrl::MPCConstraints;
using MPCTarget           = cav_ctrl::MPCTarget;
using MPCSolution         = cav_ctrl::MPCSolution;
using FeedForwardConfig   = cav_ctrl::FeedForwardConfig;
using TurbineEfficiencyModel = cav_ctrl::TurbineEfficiencyModel;
using CavitationRiskModel = cav_ctrl::CavitationRiskModel;
using ModelPredictiveController = cav_ctrl::ModelPredictiveController;

static constexpr auto MPC_STATE_DIM = cav_ctrl::MPC_STATE_DIM;
static constexpr auto MPC_INPUT_DIM = cav_ctrl::MPC_INPUT_DIM;

}
