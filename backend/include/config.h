#pragma once

#include <string>
#include <cstdint>

namespace turbine_monitor {

constexpr uint8_t  TURBINE_COUNT          = 6;
constexpr uint8_t  HYDROPHONE_COUNT       = 12;
constexpr uint8_t  ACCELEROMETER_COUNT    = 8;
constexpr uint8_t  BLADE_COUNT            = 15;
constexpr uint32_t SAMPLE_RATE_HYDROPHONE = 51200;
constexpr uint32_t SAMPLE_RATE_ACCEL      = 25600;
constexpr uint32_t SAMPLES_PER_MS_HYDRO   = 52;
constexpr uint32_t SAMPLES_PER_MS_ACCEL   = 26;
constexpr uint16_t UDP_PORT               = 9000;
constexpr uint32_t UDP_BUFFER_SIZE        = 65536;
constexpr uint32_t PROCESS_THREAD_COUNT   = 8;
constexpr uint32_t BATCH_WRITE_SIZE       = 1000;

struct Config {
    std::string clickhouse_host = "127.0.0.1";
    uint16_t    clickhouse_port = 8123;
    std::string clickhouse_user = "default";
    std::string clickhouse_pass = "";
    std::string clickhouse_db   = "turbine_monitor";

    uint16_t    udp_port        = UDP_PORT;
    std::string udp_host        = "0.0.0.0";

    uint16_t    api_port        = 8080;
    std::string api_host        = "0.0.0.0";

    std::string iec61850_server = "127.0.0.1";
    uint16_t    iec61850_port   = 102;

    float       cavitation_threshold_warning  = 0.3f;
    float       cavitation_threshold_critical = 0.6f;
    float       vibration_threshold_warning   = 3.0f;
    float       vibration_threshold_critical  = 5.0f;
    float       life_threshold_critical       = 0.8f;

    uint32_t    feature_extraction_interval_ms = 100;
    uint32_t    cavitation_detection_interval_ms = 100;
    uint32_t    life_assessment_interval_ms    = 1000;
    uint32_t    alarm_check_interval_ms        = 500;

    int         wavelet_decomposition_level    = 5;
    std::string wavelet_basis                  = "db4";

    std::string autoencoder_model_path         = "models/autoencoder.onnx";
    std::string isolation_forest_model_path    = "models/isolation_forest.json";

    bool        enable_iec61850_push           = false;
    bool        enable_autoencoder             = true;
    bool        enable_isolation_forest        = true;

    struct MPCConfig {
        bool          enabled                    = true;
        ControlMode   default_mode             = ControlMode::MPC_OPTIMAL;
        uint32_t    prediction_horizon         = 20;
        uint32_t    control_horizon            = 5;
        float         sample_time_s             = 0.1f;
        float         weight_efficiency           = 1.0f;
        float         weight_cavitation_risk    = 3.0f;
        float         weight_power_tracking     = 2.0f;
        float         weight_control_rate       = 0.5f;
        float         guide_vane_min_deg          = 15.0f;
        float         guide_vane_max_deg         = 85.0f;
        float         guide_vane_rate_limit_dps = 5.0f;
        float         power_min_mw             = 100.0f;
        float         power_max_mw             = 750.0f;
        float         power_rate_limit_mwps     = 20.0f;
        float         cavitation_safe_threshold   = 0.4f;
        float         cavitation_emergency_threshold = 0.75f;
        uint32_t    mpc_max_iterations           = 50;
        float         mpc_tolerance                = 1e-4f;
    } mpc;

    struct RobotConfig {
        bool           enabled                    = true;
        float         auto_trigger_life_pct     = 10.0f;
        float         damage_threshold_mm2       = 50.0f;
        float         robot_speed_mm_s            = 50.0f;
        float         weld_speed_mm_s            = 8.0f;
        float         polish_speed_mm_s         = 20.0f;
        float         weld_bead_width_mm      = 6.0f;
        float         polish_step_mm            = 0.5f;
        float         inspection_grid_step_mm    = 10.0f;
        uint32_t     path_waypoint_count = 64;
        float         turbine_inner_radius_mm     = 2500.0f;
        float         turbine_outer_radius_mm    = 5000.0f;
        float         safety_clearance_mm        = 50.0f;
        RepairMode  default_repair_mode     = RepairMode::POLISH_AND_WELD;
    } robot;

    struct SchedulerConfig {
        bool           enabled                    = true;
        uint32_t    scheduling_interval_s     = 300;
        uint64_t    horizon_steps              = 24;
        uint32_t    time_step_s             = 3600;
        float         weight_efficiency           = 2.0f;
        float         weight_cavitation_penalty   = 5.0f;
        float         weight_startup_cost      = 1.0f;
        float         min_up_time_s          = 14400;
        float         min_down_time_s        = 7200;
        float         max_cavitation_allow = 0.5f;
        float         reserve_margin_pct       = 5.0f;
        float         ramp_rate_limit_mwps   = 10.0f;
        uint32_t    mip_max_threads           = 4;
        uint32_t    mip_time_limit_ms       = 5000;
        float         mip_gap_tolerance      = 0.001f;
    } scheduler;

    struct DiagnosisConfig {
        bool           enabled                    = true;
        uint32_t    embedding_dim              = 32;
        uint32_t    max_patterns_per_type      = 100;
        uint32_t    update_interval_s       = 60;
        float         similarity_threshold       = 0.75f;
        float         novelty_threshold          = 0.15f;
        float         triplet_margin             = 0.3f;
        uint32_t    max_clusters               = 10;
        uint32_t    kmeans_max_iter            = 300;
        float         cluster_merge_threshold     = 0.5f;
        uint32_t    min_samples_per_cluster   = 5;
        std::string  pattern_library_path     = "config/models/acoustic_patterns.json";
    } diagnosis;
};

Config& getConfig();
bool loadConfig(const std::string& configFile);

}
