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
