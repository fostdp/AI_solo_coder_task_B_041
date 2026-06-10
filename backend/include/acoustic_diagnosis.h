#pragma once

#include "acoustic_fingerprint.h"

namespace turbine_monitor {

using Embedding             = acoustic_fp::Embedding;
using FeatureVector         = acoustic_fp::FeatureVector;
using Triplet               = acoustic_fp::Triplet;
using GPUAccelConfig        = acoustic_fp::GPUAccelConfig;
using DenoiseConfig         = acoustic_fp::DenoiseConfig;
using Spectrum              = acoustic_fp::Spectrum;
using GPUFeatureExtractor   = acoustic_fp::GPUFeatureExtractor;
using EmbeddingNetwork      = acoustic_fp::EmbeddingNetwork;
using TripletMiner          = acoustic_fp::TripletMiner;
using Cluster               = acoustic_fp::Cluster;
using OnlineKMeansClustering = acoustic_fp::OnlineKMeansClustering;
using PatternMatchResult    = acoustic_fp::PatternMatchResult;
using PatternLibrary        = acoustic_fp::PatternLibrary;
using AcousticDiagnosisFacade = acoustic_fp::AcousticFingerprintFacade;

static constexpr auto FEATURE_DIM  = acoustic_fp::FEATURE_DIM;
static constexpr auto EMBEDDING_DIM = acoustic_fp::EMBEDDING_DIM;

}
