# 大型水电站水轮机空化噪声监测与寿命评估系统

## 项目概述

本系统是一套完整的水轮机空化噪声监测与寿命评估全栈应用，专为大型水电站设计。系统通过布设的水听器和加速度计实时采集空化噪声和振动数据，基于先进的信号处理和机器学习算法实现空化状态在线识别与叶片疲劳寿命评估。

## 系统架构

```
┌─────────────────┐     UDP     ┌─────────────────┐     HTTP      ┌─────────────────┐
│  PXI 采集模拟器 │ ─────────→ │  C++ 后端服务   │ ─────────→  │  WebGL 前端      │
│  (pxi_simulator)│  数据流    │  (数据处理)     │   REST API  │  (可视化监控)    │
└─────────────────┘            └────────┬────────┘              └─────────────────┘
                                        │
                                        ▼
                              ┌─────────────────┐
                              │  ClickHouse DB  │
                              │  (时序数据存储) │
                              └─────────────────┘
```

## 主要功能

### 1. 数据采集与传输
- 支持6台混流式水轮机，每台12个水听器 + 8个加速度计
- 1ms采样间隔，水听器51.2kHz采样率，加速度计25.6kHz采样率
- UDP高速数据流传输，支持128采样点/包
- 多线程接收与处理，无锁队列保证高性能

### 2. 信号处理与特征提取
- FFT频谱分析（FFTW3加速）
- 小波包变换（db4小波，5级分解，32个频带）
- 频谱特征提取：峰值频率、RMS值、峰值因子、峭度、偏度、谱质心等
- 小波包特征：各频带能量、能量比、能量熵

### 3. 空化状态在线识别
- **孤立森林算法**：100棵树，子采样256个样本，基于路径长度计算异常分数
- **深度自编码器**：32维输入→16维隐藏→8维编码，基于重构误差检测异常
- **集成学习**：两种模型加权平均，提高检测准确率
- 空化阶段分类：正常(<0.3)、初生(0.3-0.6)、临界(0.6-0.8)、发展(>0.8)

### 4. 叶片疲劳寿命评估
- **雨流计数法**：四点法识别应力循环
- **Goodman修正**：考虑平均应力对疲劳寿命的影响
- **Miner线性累积损伤理论**：D = Σ(ni/Ni)
- 剩余寿命估算：基于历史损伤速率预测

### 5. 告警系统
- 空化强度超限告警
- 叶片振动超标告警
- 寿命预警
- IEC 61850协议推送至监控系统
- 智能检修建议生成
- 告警抑制与确认机制

### 6. 可视化前端
- **WebGL水轮机剖面图**：蜗壳、导叶、转轮、尾水管三维模型
- **空化强度云图**：颜色映射叠加在转轮叶片上
- **瀑布图**：噪声频谱实时滚动显示
- **交互功能**：点击叶片查看历史趋势和损伤详情
- **告警面板**：实时告警展示与管理

## 目录结构

```
AI_solo_coder_task_A_041/
├── backend/                    # C++后端代码
│   ├── include/                # 头文件
│   │   ├── config.h            # 配置管理
│   │   ├── data_structures.h   # 数据结构定义
│   │   ├── udp_server.h        # UDP服务器
│   │   ├── clickhouse_client.h # ClickHouse客户端
│   │   ├── signal_processor.h  # 信号处理器
│   │   ├── cavitation_detector.h  # 空化检测器
│   │   ├── life_assessor.h     # 寿命评估器
│   │   ├── alarm_manager.h     # 告警管理器
│   │   ├── api_server.h        # API服务器
│   │   └── data_pipeline.h     # 数据管道
│   ├── src/                    # 源文件
│   │   ├── main.cpp            # 主程序入口
│   │   ├── config.cpp
│   │   ├── udp_server.cpp
│   │   ├── clickhouse_client.cpp
│   │   ├── signal_processor.cpp
│   │   ├── cavitation_detector.cpp
│   │   ├── life_assessor.cpp
│   │   ├── alarm_manager.cpp
│   │   ├── api_server.cpp
│   │   └── data_pipeline.cpp
│   └── CMakeLists.txt          # 构建配置
├── frontend/                   # 前端代码
│   ├── index.html              # 主页面
│   ├── css/
│   │   └── style.css           # 样式文件
│   └── js/
│       ├── turbine_viewer.js   # WebGL水轮机视图
│       ├── waterfall_chart.js  # 瀑布图组件
│       └── main.js             # 主逻辑
├── simulator/                  # 模拟器
│   └── pxi_simulator.py        # PXI采集模拟器
├── clickhouse/                 # 数据库
│   └── init.sql                # 初始化脚本
├── config/                     # 配置文件
│   └── config.json             # 系统配置
└── README.md                   # 本文件
```

## 快速开始

### 1. 环境要求

**后端依赖：**
- C++17 编译器 (GCC 7+ / Clang 5+ / MSVC 2017+)
- CMake 3.10+
- FFTW3 (可选，用于FFT加速)
- libcurl (可选，用于ClickHouse HTTP接口)
- jsoncpp (可选，用于JSON解析)
- pthread (Linux) / WS2_32 (Windows)

**前端依赖：**
- 现代浏览器（支持WebGL）
- gl-matrix.js（已通过CDN引入）

**数据库：**
- ClickHouse 21.8+

### 2. 数据库初始化

```bash
# 启动ClickHouse服务
clickhouse-server --config-file=/etc/clickhouse-server/config.xml &

# 执行初始化脚本
clickhouse-client --multiquery < clickhouse/init.sql
```

### 3. 后端编译

```bash
cd backend
mkdir build && cd build
cmake ..
make -j$(nproc)

# 运行
./turbine_monitor -c ../../config/config.json
```

### 4. 启动模拟器

```bash
cd simulator
pip install numpy
python3 pxi_simulator.py -H 127.0.0.1 -p 9000
```

### 5. 启动前端

```bash
cd frontend
python3 -m http.server 8000

# 浏览器访问 http://localhost:8000
```

## 核心技术指标

| 指标 | 数值 |
|------|------|
| 监测机组数 | 6台 |
| 每台水听器 | 12个 |
| 每台加速度计 | 8个 |
| 采样率（水听器） | 51.2 kHz |
| 采样率（加速度计） | 25.6 kHz |
| 上报间隔 | 1 ms |
| 数据包速率 | 120,000 包/秒 |
| 数据吞吐量 | ~120 Mbps |
| 处理延迟 | < 10 ms |
| 叶片数/每台 | 15个 |

## ClickHouse 表结构

系统设计了11张核心表和2个物化视图：

- `raw_sensor_data` - 原始传感器数据（TTL 30天）
- `spectrum_features` - 频谱特征（TTL 90天）
- `wavelet_features` - 小波包特征（TTL 90天）
- `cavitation_state` - 空化状态识别结果（TTL 1年）
- `blade_stress` - 叶片应力计算结果（TTL 1年）
- `life_assessment` - 寿命评估结果（TTL 3年）
- `alarm_logs` - 告警记录（TTL 3年）
- `turbine_config` - 水轮机配置
- `sensor_config` - 传感器配置
- `cavitation_intensity_1s_table` - 1秒聚合（物化视图）
- `cavitation_damage_daily_table` - 日损伤累计（物化视图）

## API 接口

| 接口 | 方法 | 描述 |
|------|------|------|
| `/api/turbines` | GET | 获取所有水轮机列表 |
| `/api/turbine/detail` | GET | 获取水轮机详情 |
| `/api/cavitation` | GET | 获取空化状态数据 |
| `/api/life` | GET | 获取寿命评估数据 |
| `/api/spectrum` | GET | 获取频谱数据 |
| `/api/waterfall` | GET | 获取瀑布图数据 |
| `/api/alarms/active` | GET | 获取活跃告警 |
| `/api/alarms/acknowledge` | POST | 确认告警 |
| `/api/alarms/suppress` | POST | 抑制告警 |
| `/api/status` | GET | 获取系统状态 |

## 关键算法

### 空化检测算法

```
异常分数 = 0.5 * IF_score + 0.5 * AE_reconstruction_error

IF_score = 2^(-avgPathLength / cFactor(n))
AE_error = MSE(original, reconstructed)

阈值分类:
  < 0.3 → 正常
0.3-0.6 → 初生空化
0.6-0.8 → 临界空化
  > 0.8 → 发展空化
```

### 寿命评估算法

```
雨流计数 → 应力循环识别
Goodman修正 → σ_a' = σ_a / (1 - σ_m / σ_uts)
Miner损伤 → D = Σ(ni / Ni)
Ni = k * (Δσ)^(-m)
剩余寿命 = (1 - D) / (dD/dt)
```

## 性能优化

1. **多线程架构**：1个接收线程 + 8个处理线程
2. **无锁队列**：TBB并发队列实现高效数据传递
3. **批量写入**：ClickHouse批量1000条或1秒自动刷新
4. **FFTW3加速**：快速傅里叶变换性能优化
5. **SIMD指令**：编译器自动向量化
6. **内存池**：减少内存分配开销
7. **TTL策略**：自动清理过期数据
8. **物化视图**：预聚合常用查询

## 监控与运维

系统提供完整的运行状态监控：
- 实时统计：接收包数、处理包数、队列长度、处理延迟
- 日志分级：DEBUG/INFO/WARNING/ERROR
- 健康检查接口：`/api/status`
- 内置性能计数器

## 扩展计划

- [ ] 支持更多传感器类型（压力、温度等）
- [ ] 深度学习模型在线训练
- [ ] 数字孪生集成
- [ ] 多电站集中监控
- [ ] 移动端APP
- [ ] 边缘计算节点支持

## 技术支持

如有问题，请联系系统运维团队。

---

**版本**: 1.0.0  
**更新日期**: 2026-06-10  
**适用场景**: 大型混流式水轮机组空化监测与寿命评估
