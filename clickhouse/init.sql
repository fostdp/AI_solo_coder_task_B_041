-- =========================================
-- 大型水电站水轮机空化噪声监测与寿命评估系统
-- ClickHouse 数据库初始化脚本
-- =========================================

CREATE DATABASE IF NOT EXISTS turbine_monitor;

USE turbine_monitor;

-- =========================================
-- 1. 原始数据表 - 每毫秒采集的原始数据
-- =========================================
CREATE TABLE IF NOT EXISTS raw_sensor_data (
    timestamp UInt64,
    turbine_id UInt8,
    sensor_type Enum8('hydrophone' = 1, 'accelerometer' = 2),
    sensor_id UInt8,
    sensor_position Enum8('spiral_inlet' = 1, 'guide_vane' = 2, 'draft_tube' = 3, 'runner_blade' = 4),
    blade_id UInt8,
    amplitude Float32,
    raw_data Array(Float32),
    sample_rate UInt32,
    batch_id UInt64
) ENGINE = MergeTree()
PARTITION BY toDate(timestamp / 1000)
ORDER BY (turbine_id, sensor_type, sensor_id, timestamp)
TTL toDateTime(timestamp / 1000) + INTERVAL 30 DAY
SETTINGS index_granularity = 8192;

-- =========================================
-- 2. 频谱特征表 - FFT分析结果
-- =========================================
CREATE TABLE IF NOT EXISTS spectrum_features (
    timestamp UInt64,
    turbine_id UInt8,
    sensor_type Enum8('hydrophone' = 1, 'accelerometer' = 2),
    sensor_id UInt8,
    blade_id UInt8,
    peak_frequency Float32,
    rms_value Float32,
    crest_factor Float32,
    kurtosis Float32,
    skewness Float32,
    band_energy_low Float32,
    band_energy_mid Float32,
    band_energy_high Float32,
    harmonic_ratio Float32,
    spectral_centroid Float32,
    spectral_bandwidth Float32
) ENGINE = MergeTree()
PARTITION BY toDate(timestamp / 1000)
ORDER BY (turbine_id, sensor_id, timestamp)
TTL toDateTime(timestamp / 1000) + INTERVAL 90 DAY
SETTINGS index_granularity = 4096;

-- =========================================
-- 3. 小波包能量特征表
-- =========================================
CREATE TABLE IF NOT EXISTS wavelet_features (
    timestamp UInt64,
    turbine_id UInt8,
    sensor_id UInt8,
    blade_id UInt8,
    wavelet_basis String,
    decomposition_level UInt8,
    band_energy Array(Float32),
    energy_entropy Float32,
    energy_ratio Array(Float32),
    total_energy Float32
) ENGINE = MergeTree()
PARTITION BY toDate(timestamp / 1000)
ORDER BY (turbine_id, sensor_id, timestamp)
TTL toDateTime(timestamp / 1000) + INTERVAL 90 DAY
SETTINGS index_granularity = 4096;

-- =========================================
-- 4. 空化状态识别结果表
-- =========================================
CREATE TABLE IF NOT EXISTS cavitation_state (
    timestamp UInt64,
    turbine_id UInt8,
    blade_id UInt8,
    cavitation_stage Enum8('normal' = 0, 'incipient' = 1, 'critical' = 2, 'developed' = 3),
    cavitation_intensity Float32,
    confidence Float32,
    model_type Enum8('autoencoder' = 1, 'isolation_forest' = 2, 'ensemble' = 3),
    anomaly_score Float32,
    reconstruction_error Float32,
    feature_vector Array(Float32)
) ENGINE = MergeTree()
PARTITION BY toDate(timestamp / 1000)
ORDER BY (turbine_id, blade_id, timestamp)
TTL toDateTime(timestamp / 1000) + INTERVAL 365 DAY
SETTINGS index_granularity = 2048;

-- =========================================
-- 5. 叶片应力与疲劳数据表
-- =========================================
CREATE TABLE IF NOT EXISTS blade_stress (
    timestamp UInt64,
    turbine_id UInt8,
    blade_id UInt8,
    vibration_stress Float32,
    cavitation_stress Float32,
    combined_stress Float32,
    stress_cycles UInt32,
    max_stress Float32,
    min_stress Float32,
    mean_stress Float32,
    stress_amplitude Float32,
    rainflow_cycles Array(Float32)
) ENGINE = MergeTree()
PARTITION BY toDate(timestamp / 1000)
ORDER BY (turbine_id, blade_id, timestamp)
TTL toDateTime(timestamp / 1000) + INTERVAL 365 DAY
SETTINGS index_granularity = 2048;

-- =========================================
-- 6. 寿命评估结果表
-- =========================================
CREATE TABLE IF NOT EXISTS life_assessment (
    timestamp UInt64,
    turbine_id UInt8,
    blade_id UInt8,
    cumulative_damage Float32,
    remaining_life_hours Float32,
    remaining_life_days Float32,
    miner_sum Float32,
    fatigue_damage Float32,
    cavitation_damage Float32,
    material_constant_k Float32,
    material_constant_m Float32,
    stress_range Float32,
    cycle_count UInt32,
    assessment_method String
) ENGINE = MergeTree()
PARTITION BY toDate(timestamp / 1000)
ORDER BY (turbine_id, blade_id, timestamp)
TTL toDateTime(timestamp / 1000) + INTERVAL 365 DAY
SETTINGS index_granularity = 1024;

-- =========================================
-- 7. 告警记录表
-- =========================================
CREATE TABLE IF NOT EXISTS alarm_logs (
    timestamp UInt64,
    alarm_id UUID,
    turbine_id UInt8,
    blade_id UInt8,
    alarm_type Enum8('cavitation_exceed' = 1, 'vibration_exceed' = 2, 'life_critical' = 3, 'sensor_fault' = 4),
    alarm_level Enum8('warning' = 1, 'critical' = 2, 'emergency' = 3),
    alarm_message String,
    threshold_value Float32,
    actual_value Float32,
    iec61850_pushed Bool,
    acknowledged Bool,
    maintenance_suggestion String,
    acknowledged_at UInt64,
    acknowledged_by String
) ENGINE = MergeTree()
PARTITION BY toDate(timestamp / 1000)
ORDER BY (turbine_id, alarm_level, timestamp)
TTL toDateTime(timestamp / 1000) + INTERVAL 730 DAY
SETTINGS index_granularity = 1024;

-- =========================================
-- 8. 水轮机配置表
-- =========================================
CREATE TABLE IF NOT EXISTS turbine_config (
    turbine_id UInt8 PRIMARY KEY,
    turbine_name String,
    turbine_type String DEFAULT 'Francis',
    rated_power Float32,
    rated_head Float32,
    rated_flow Float32,
    rated_speed Float32,
    blade_count UInt8 DEFAULT 15,
    material String DEFAULT '13Cr4Ni',
    ultimate_tensile_strength Float32,
    fatigue_limit Float32,
    fracture_toughness Float32,
    cavitation_threshold Float32,
    vibration_threshold Float32,
    expected_life_hours Float32
) ENGINE = ReplacingMergeTree()
ORDER BY turbine_id;

-- =========================================
-- 9. 传感器配置表
-- =========================================
CREATE TABLE IF NOT EXISTS sensor_config (
    turbine_id UInt8,
    sensor_type Enum8('hydrophone' = 1, 'accelerometer' = 2),
    sensor_id UInt8,
    sensor_position Enum8('spiral_inlet' = 1, 'guide_vane' = 2, 'draft_tube' = 3, 'runner_blade' = 4),
    blade_id UInt8,
    calibration_factor Float32,
    sensitivity Float32,
    sampling_rate UInt32,
    range_min Float32,
    range_max Float32,
    install_date Date,
    last_calibration Date,
    PRIMARY KEY (turbine_id, sensor_type, sensor_id)
) ENGINE = ReplacingMergeTree()
ORDER BY (turbine_id, sensor_type, sensor_id);

-- =========================================
-- 10. 物化视图 - 空化强度1秒聚合
-- =========================================
CREATE MATERIALIZED VIEW IF NOT EXISTS cavitation_intensity_1s
TO cavitation_intensity_1s_table
AS SELECT
    toStartOfSecond(toDateTime(timestamp / 1000)) AS time_1s,
    turbine_id,
    blade_id,
    avg(cavitation_intensity) AS avg_intensity,
    max(cavitation_intensity) AS max_intensity,
    quantile(0.95)(cavitation_intensity) AS p95_intensity
FROM cavitation_state
GROUP BY time_1s, turbine_id, blade_id;

CREATE TABLE IF NOT EXISTS cavitation_intensity_1s_table (
    time_1s DateTime,
    turbine_id UInt8,
    blade_id UInt8,
    avg_intensity Float32,
    max_intensity Float32,
    p95_intensity Float32
) ENGINE = AggregatingMergeTree()
PARTITION BY toDate(time_1s)
ORDER BY (time_1s, turbine_id, blade_id)
TTL time_1s + INTERVAL 365 DAY;

-- =========================================
-- 11. 物化视图 - 空化损伤日累计
-- =========================================
CREATE MATERIALIZED VIEW IF NOT EXISTS cavitation_damage_daily
TO cavitation_damage_daily_table
AS SELECT
    toDate(timestamp / 1000) AS date,
    turbine_id,
    blade_id,
    sum(cavitation_damage) AS total_damage,
    avg(cumulative_damage) AS avg_cumulative_damage,
    max(cumulative_damage) AS max_cumulative_damage
FROM life_assessment
GROUP BY date, turbine_id, blade_id;

CREATE TABLE IF NOT EXISTS cavitation_damage_daily_table (
    date Date,
    turbine_id UInt8,
    blade_id UInt8,
    total_damage Float32,
    avg_cumulative_damage Float32,
    max_cumulative_damage Float32
) ENGINE = AggregatingMergeTree()
PARTITION BY date
ORDER BY (date, turbine_id, blade_id)
TTL date + INTERVAL 1095 DAY;

-- =========================================
-- 插入默认配置数据
-- =========================================
INSERT INTO turbine_config VALUES
(1, '1号水轮机', 'Francis', 700.0, 120.0, 650.0, 107.1, 15, '13Cr4Ni', 750.0, 250.0, 60.0, 0.5, 5.0, 200000.0),
(2, '2号水轮机', 'Francis', 700.0, 120.0, 650.0, 107.1, 15, '13Cr4Ni', 750.0, 250.0, 60.0, 0.5, 5.0, 200000.0),
(3, '3号水轮机', 'Francis', 700.0, 120.0, 650.0, 107.1, 15, '13Cr4Ni', 750.0, 250.0, 60.0, 0.5, 5.0, 200000.0),
(4, '4号水轮机', 'Francis', 700.0, 120.0, 650.0, 107.1, 15, '13Cr4Ni', 750.0, 250.0, 60.0, 0.5, 5.0, 200000.0),
(5, '5号水轮机', 'Francis', 700.0, 120.0, 650.0, 107.1, 15, '13Cr4Ni', 750.0, 250.0, 60.0, 0.5, 5.0, 200000.0),
(6, '6号水轮机', 'Francis', 700.0, 120.0, 650.0, 107.1, 15, '13Cr4Ni', 750.0, 250.0, 60.0, 0.5, 5.0, 200000.0);

-- 插入水听器配置 (每台12个)
INSERT INTO sensor_config (turbine_id, sensor_type, sensor_id, sensor_position, blade_id, calibration_factor, sensitivity, sampling_rate, range_min, range_max, install_date, last_calibration)
SELECT
    t,
    'hydrophone',
    s,
    CASE
        WHEN s <= 4 THEN 'spiral_inlet'
        WHEN s <= 8 THEN 'guide_vane'
        ELSE 'draft_tube'
    END,
    0,
    1.0,
    0.001,
    51200,
    -10.0,
    10.0,
    '2024-01-01',
    '2026-01-01'
FROM numbers(1, 6) AS t, numbers(1, 12) AS s;

-- 插入加速度计配置 (每台8个)
INSERT INTO sensor_config (turbine_id, sensor_type, sensor_id, sensor_position, blade_id, calibration_factor, sensitivity, sampling_rate, range_min, range_max, install_date, last_calibration)
SELECT
    t,
    'accelerometer',
    s,
    'runner_blade',
    s,
    1.0,
    0.01,
    25600,
    -50.0,
    50.0,
    '2024-01-01',
    '2026-01-01'
FROM numbers(1, 6) AS t, numbers(1, 8) AS s;

-- =========================================
-- 12. MPC水轮机调节控制日志表 (Feature 1)
-- =========================================
CREATE TABLE IF NOT EXISTS mpc_control_logs (
    timestamp UInt64,
    turbine_id UInt8,
    control_mode Enum8('manual' = 0, 'efficiency_only' = 1, 'cavitation_safe' = 2, 'mpc_optimal' = 3),
    cavitation_avoidance_enabled Bool,
    guide_vane_opening_deg Float32,
    target_power_mw Float32,
    current_head_m Float32,
    current_flow_m3s Float32,
    predicted_efficiency Float32,
    predicted_cavitation_risk Float32,
    mpc_cost_value Float32,
    control_signals Array(Float32),
    horizon_states Array(Float32),
    control_action_desc String
) ENGINE = MergeTree()
PARTITION BY toDate(timestamp / 1000)
ORDER BY (turbine_id, timestamp)
TTL toDateTime(timestamp / 1000) + INTERVAL 365 DAY
SETTINGS index_granularity = 2048;

-- =========================================
-- 13. 水下机器人检修任务表 (Feature 2)
-- =========================================
CREATE TABLE IF NOT EXISTS robot_repair_tasks (
    timestamp UInt64,
    task_id UUID,
    turbine_id UInt8,
    robot_status Enum8('idle' = 0, 'planning' = 1, 'deploying' = 2, 'inspecting' = 3,
                       'polishing' = 4, 'welding' = 5, 'returning' = 6, 'completed' = 7, 'fault' = 8),
    repair_mode Enum8('inspection_only' = 0, 'polish' = 1, 'weld' = 2, 'polish_and_weld' = 3),
    target_blade_ids Array(UInt8),
    estimated_duration_ms UInt64,
    total_repair_area_cm2 Float32,
    total_weld_volume_cm3 Float32,
    inspection_path Array(Float32),
    polish_trajectory Array(Float32),
    weld_trajectory Array(Float32),
    blade_damage_map Array(Float32),
    repair_sequence String,
    robot_current_pos Array(Float32),
    current_waypoint_idx UInt32,
    completed_at UInt64,
    expert_approved Bool DEFAULT 0,
    approved_by String DEFAULT ''
) ENGINE = ReplacingMergeTree(completed_at)
PARTITION BY toYYYYMM(toDateTime(timestamp / 1000))
ORDER BY (task_id, timestamp)
TTL toDateTime(timestamp / 1000) + INTERVAL 1095 DAY;

CREATE MATERIALIZED VIEW IF NOT EXISTS robot_task_daily_mv
TO robot_task_daily_table
AS SELECT
    toDate(timestamp / 1000) AS date,
    turbine_id,
    robot_status,
    count() AS task_count,
    sum(total_repair_area_cm2) AS total_area,
    avg(estimated_duration_ms) AS avg_duration
FROM robot_repair_tasks
GROUP BY date, turbine_id, robot_status;

CREATE TABLE IF NOT EXISTS robot_task_daily_table (
    date Date,
    turbine_id UInt8,
    robot_status UInt8,
    task_count UInt64,
    total_area Float32,
    avg_duration Float64
) ENGINE = AggregatingMergeTree()
PARTITION BY date
ORDER BY (date, turbine_id, robot_status)
TTL date + INTERVAL 1095 DAY;

-- =========================================
-- 14. 全厂优化调度结果表 (Feature 3)
-- =========================================
CREATE TABLE IF NOT EXISTS plant_schedules (
    timestamp UInt64,
    schedule_id UInt8,
    optimization_status Enum8('not_optimized' = 0, 'optimizing' = 1, 'converged' = 2,
                              'partial_feasible' = 3, 'infeasible' = 4),
    scheduling_horizon_s UInt64,
    target_total_power_mw Float32,
    current_total_power_mw Float32,
    optimized_efficiency_pct Float32,
    cavitation_risk_reduction_pct Float32,
    unit_power_mw Array(Float32),
    unit_efficiency Array(Float32),
    unit_cavitation_risk Array(Float32),
    unit_operating_hours Array(Float32),
    mip_objective_value Float32,
    constraint_slack Array(Float32),
    schedule_note String,
    executed Bool DEFAULT 0
) ENGINE = MergeTree()
PARTITION BY toDate(timestamp / 1000)
ORDER BY (schedule_id, timestamp)
TTL toDateTime(timestamp / 1000) + INTERVAL 730 DAY
SETTINGS index_granularity = 1024;

CREATE MATERIALIZED VIEW IF NOT EXISTS schedule_hourly_stats_mv
TO schedule_hourly_stats_table
AS SELECT
    toStartOfHour(toDateTime(timestamp / 1000)) AS hour,
    avg(optimized_efficiency_pct) AS avg_efficiency,
    avg(cavitation_risk_reduction_pct) AS avg_cav_reduction,
    avg(current_total_power_mw) AS avg_power,
    sum(target_total_power_mw) AS total_target
FROM plant_schedules
WHERE optimization_status = 2
GROUP BY hour;

CREATE TABLE IF NOT EXISTS schedule_hourly_stats_table (
    hour DateTime,
    avg_efficiency Float64,
    avg_cav_reduction Float64,
    avg_power Float64,
    total_target Float64
) ENGINE = AggregatingMergeTree()
PARTITION BY toDate(hour)
ORDER BY hour
TTL hour + INTERVAL 1095 DAY;

-- =========================================
-- 15. 声纹特征库与诊断结果表 (Feature 4)
-- =========================================
CREATE TABLE IF NOT EXISTS acoustic_patterns (
    timestamp UInt64,
    pattern_id UUID,
    cavitation_type Enum8('unknown' = 0, 'cloud' = 1, 'sheet' = 2, 'super' = 3,
                          'vortex' = 4, 'tip_leakage' = 5),
    pattern_name String,
    description String,
    embedding Array(Float32),
    centroid Array(Float32),
    sample_count UInt32,
    intra_cluster_variance Float32,
    silhouette_score Float32,
    is_verified_by_expert Bool,
    expert_note String,
    last_updated UInt64
) ENGINE = ReplacingMergeTree(last_updated)
ORDER BY (pattern_id, cavitation_type)
TTL toDateTime(timestamp / 1000) + INTERVAL 1095 DAY;

CREATE TABLE IF NOT EXISTS diagnosis_results (
    timestamp UInt64,
    turbine_id UInt8,
    sensor_id UInt8,
    cavitation_type Enum8('unknown' = 0, 'cloud' = 1, 'sheet' = 2, 'super' = 3,
                          'vortex' = 4, 'tip_leakage' = 5),
    diagnosis_status Enum8('pending' = 0, 'extracting' = 1, 'clustering' = 2,
                           'matching' = 3, 'completed' = 4, 'needs_expert' = 5),
    cluster_label UInt8,
    is_known_pattern Bool,
    feature_embedding Array(Float32),
    pattern_similarity Array(Float32),
    confidence_scores Array(Float32),
    centroid_distance Float32,
    silhouette_score Float32,
    cluster_purity Float32,
    cavitation_type_name String,
    expert_note String,
    analysis_latency_us UInt64
) ENGINE = MergeTree()
PARTITION BY toDate(timestamp / 1000)
ORDER BY (turbine_id, sensor_id, timestamp)
TTL toDateTime(timestamp / 1000) + INTERVAL 730 DAY
SETTINGS index_granularity = 2048;

CREATE MATERIALIZED VIEW IF NOT EXISTS diagnosis_daily_summary_mv
TO diagnosis_daily_summary_table
AS SELECT
    toDate(timestamp / 1000) AS date,
    cavitation_type,
    is_known_pattern,
    count() AS diagnosis_count,
    avg(silhouette_score) AS avg_silhouette,
    avg(centroid_distance) AS avg_distance
FROM diagnosis_results
WHERE diagnosis_status = 4
GROUP BY date, cavitation_type, is_known_pattern;

CREATE TABLE IF NOT EXISTS diagnosis_daily_summary_table (
    date Date,
    cavitation_type UInt8,
    is_known_pattern Bool,
    diagnosis_count UInt64,
    avg_silhouette Float64,
    avg_distance Float64
) ENGINE = AggregatingMergeTree()
PARTITION BY date
ORDER BY (date, cavitation_type, is_known_pattern)
TTL date + INTERVAL 1095 DAY;
