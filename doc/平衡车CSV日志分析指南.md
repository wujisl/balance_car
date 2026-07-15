# 平衡车 CSV 日志分析指南

本指南对应上位机保存的 `tool/records/balance_speed_*.csv`。日志是主板的结构化遥测快照，适合定位传感器、姿态、平衡环、速度环、转向和里程计问题；不要把它当作电机驱动的逐控制周期追踪记录。

## 1. 先做数据有效性检查

1. 用 `board_timestamp_ms` 排序；它是主板 `millis()` 时钟，优先级高于电脑 `timestamp`。
2. 计算相邻行 `dt = diff(board_timestamp_ms) / 1000`。正常遥测约为 50 Hz，即约 `0.020 s`；Wi-Fi 抖动可造成偶发间隔增大。
3. 检查 `sequence` 是否连续。差值大于 1 表示 UDP 包丢失；不要用缺失期间的两行直接估计尖峰角速度或加速度。
4. 同时检查 `safety_state`、`fault_code` 和 `imu_valid`。只有 `BALANCING` 且 `imu_valid=1` 的区段可用于控制效果分析。
5. 参数列是每帧的实测回读值。一次 CSV 中若参数改变，应按参数变化点分段比较，不能混为一段得出结论。

安全状态编码为：`0 BOOT`、`1 SELF_TESTING`、`2 STANDBY`、`3 MANUAL_TEST`、`4 BALANCING`、`5 FAULT`。

故障编码为：`0 NONE`、`1 SELF_TEST_FAILED`、`2 IMU_UNHEALTHY`、`3 PITCH_LIMIT_EXCEEDED`、`4 AIRBORNE_LANDING_FAILED`。

## 2. 字段速查表

| 字段 | 单位 / 含义 | 分析要点 |
| --- | --- | --- |
| `timestamp` | 电脑记录时间 | 用于对照视频、人工操作和系统日志；不用于控制周期计算。 |
| `board_timestamp_ms` | ms，主板启动后的时间 | 主时间轴；重启后会回到较小数值。 |
| `sequence` | 遥测包序号 | 检测 UDP 丢包和乱序。 |
| `safety_state`、`fault_code`、`imu_valid` | 状态码 / 0 或 1 | 故障发生点首先看这三列。 |
| `pitch_deg` | °，互补滤波后的俯仰角 | 平衡环实际使用的姿态。 |
| `pitch_rate_dps` | °/s，俯仰轴陀螺角速度 | D 项使用的角速度；快速变化时应与 `pitch_deg` 的变化方向一致。 |
| `accelerometer_pitch_deg` | °，仅由加速度计推算的俯仰 | 静止时应接近 `pitch_deg`；运动/颠簸时可明显偏离，不能单独判为 IMU 坏。 |
| `accel_x_g/y_g/z_g` | g，三轴加速度 | 静止或匀速时模长应接近 1 g；冲击、加速和振动会偏离。 |
| `gyro_x_dps/y_dps/z_dps` | °/s，三轴角速度 | 用于检查安装轴、振动和转向动态；`gyro_z_dps` 可与编码器转向趋势交叉验证。 |
| `requested_pitch_deg` | ° | 平衡环目标角，约等于 `balance_trim + speed_pitch_offset`。 |
| `balance_pitch_error_deg` | ° | 定义为 `pitch_deg - requested_pitch_deg`；正负号必须结合电机方向验证。 |
| `balance_p_term/i_term/d_term` | 归一化电机命令分量 | P、I、D 对内环输出的贡献。 |
| `balance_motor_raw` | 归一化命令 | 平衡环总输出；达到最大输出时会被限幅。 |
| `left_motor_command/right_motor_command` | 归一化命令，通常 -1～1 | 混控后的最终左右轮命令；用于判断饱和和转向是否抢占平衡余量。 |
| `left_wheel_mps/right_wheel_mps` | m/s | 由左右编码器计算的轮线速度；前进时两者应同号。 |
| `balance_kp/ki/kd/trim` | 当前内环参数 | 必须与该时间段的响应一起看。 |
| `speed_kp/ki/speed_invert` | 当前速度环参数 / 方向标志 | `speed_invert=1` 时速度环输出方向被反转。 |
| `target_speed` | m/s | 前进为正、后退为负；这是已经经过 Wi-Fi 斜坡后的实际目标。 |
| `target_differential_speed_mps` | m/s，右减左目标轮速 | 正值表示右轮目标更快；按当前车体约定通常对应左转。 |
| `filtered_differential_speed_mps` | m/s，右减左实际轮速 | 转向环的反馈量，不是原始瞬时差速。 |
| `differential_speed_error_mps` | m/s | `target_differential_speed_mps - filtered_differential_speed_mps`。 |
| `turn_motor_command` | 归一化命令 | 差速控制器请求的转向输出。 |
| `applied_turn_motor_command` | 归一化命令 | 经混控剩余余量限制后实际施加的转向输出。 |
| `turn_kp/ki/max_turn/turn_invert` | 当前转向环参数 / 方向标志 | `turn_invert=1` 时转向输出方向被反转。 |
| `relative_heading_deg` | °，相对航向 | 从本次进入平衡开始积分，范围 `(-180, 180]`；不是地磁绝对航向。 |
| `yaw_rate_deg_per_s` | °/s | 由左右编码器差速计算并轻度滤波的转向角速度。 |

## 3. 控制链路与应满足的关系

### 3.1 平衡内环

控制器遵循下列关系：

```text
requested_pitch_deg ≈ balance_trim + speed_pitch_offset
balance_pitch_error_deg = pitch_deg - requested_pitch_deg
P = balance_kp * error
D = balance_kd * pitch_rate_dps
raw ≈ P + I + D
```

`balance_motor_raw` 在电机输出方向反转后记录；因此当 `motorOutputInverted` 被启用时，`raw` 的符号可能与 `P+I+D` 相反。最终输出还会受 `max_motor` 和混控限制，不能只用 `balance_motor_raw` 推断左右电机实际命令。

健康的低速平衡通常表现为：`pitch_deg` 围绕 `requested_pitch_deg` 小幅来回，`pitch_rate_dps` 不持续同号，P/D 项有正有负，且左右电机在未转向时大致相同。

### 3.2 速度外环

`target_speed` 是经斜坡后的目标，`filtered_speed` 未单独写入本 CSV，但可用 `(left_wheel_mps + right_wheel_mps) / 2` 近似检查方向和量级。速度环通过 `speed_pitch_offset` 改变 `requested_pitch_deg`，而不是直接控制电机。

对于一次正向速度阶跃，预期链路为：`target_speed` 上升 → `speed_pitch_offset` 变化 → `requested_pitch_deg` 随之变化 → 车轮平均速度同向变化。若实际速度反向，优先检查左右编码器符号、`speed_invert` 和电机方向，不要先增大 Kp。

### 3.3 差速转向与混控

```text
measured_diff = right_wheel_mps - left_wheel_mps
diff_error = target_differential_speed_mps - filtered_differential_speed_mps
turn_motor_command = turn PI 输出
applied_turn_motor_command = 受平衡输出剩余余量限制后的实际输出
left_motor_command  ≈ balance - applied_turn
right_motor_command ≈ balance + applied_turn
```

当 `abs(applied_turn_motor_command) < abs(turn_motor_command)`，说明平衡输出已占用较多电机余量；此时继续增大转向 Kp 或转向目标，可能削弱平衡能力。先降低速度或转向目标，再检查机械和参数。

### 3.4 编码器航向

航向来自差速里程计：

```text
yaw_rate ≈ (right_wheel_mps - left_wheel_mps) / wheel_track
heading[k+1] = wrap(heading[k] + yaw_rate * dt)
```

`yaw_rate_deg_per_s` 经过轻度滤波，`relative_heading_deg` 则由每个速度控制周期的编码器里程增量积分。直线行驶时两者应接近 0；转弯时二者符号应与左右轮速度差一致。轮胎打滑、轮子悬空、轮距设错和轮径/编码器标定误差都会造成航向误差。

## 4. 推荐分析流程

### A. 先定位故障时刻

筛选第一行 `safety_state=FAULT` 或 `imu_valid=0`，向前查看至少 1 秒：

- `imu_valid` 从 1 变为 0 且故障为 `IMU_UNHEALTHY`：优先检查 I²C、IMU 供电、接地、电机电磁干扰和振动；不要把加速度突变本身误认为读取失败。
- `abs(pitch_deg)` 接近或超过安全角、故障为 `PITCH_LIMIT_EXCEEDED`：查看此前 `requested_pitch_deg`、`balance_pitch_error_deg`、`pitch_rate_dps` 与电机是否饱和，判断是控制方向、增益不足还是机械失稳。
- 故障前两侧 `left_motor_command/right_motor_command` 长时间贴近限幅：先降低速度或 `max_motor`，检查是否存在机械阻力、低电压或参数过激。

### B. 检查静止基线

选取车辆不动、已直立的 3～5 秒：

- 加速度模长 `sqrt(ax²+ay²+az²)` 应大致接近 1 g。
- `pitch_deg` 与 `accelerometer_pitch_deg` 的平均差应稳定；稳定的常量差可通过 Trim 或加速度计角度偏置校准。
- `pitch_rate_dps`、`gyro_*_dps` 不应持续明显偏向一侧；若有固定偏置，重新静止校准陀螺仪。
- 两轮速度均值应接近 0；一侧持续跳动时检查该编码器信号、上拉、机械间隙或电磁干扰。

### C. 分别做小阶跃实验

每次只改变一个量，并在安全地面上从小值开始：

1. **平衡扰动**：不下发速度和转向，轻推车体。看误差出现后 P/D 项是否产生能拉回车体的电机输出；若越推越倒，先查方向而非调高增益。
2. **前/后退**：从 `±0.05 m/s` 开始。平均轮速应与 `target_speed` 同号，`speed_pitch_offset` 应有限且能在到速后回落。
3. **左/右转**：保持低速直行后施加小差速度。左右轮差、`yaw_rate_deg_per_s` 和 `relative_heading_deg` 应同号变化；若相反，检查右编码器反向配置和 `turn_invert`。
4. **停止命令**：`target_speed` 与目标差速度应逐步回到 0；若仍持续转向，检查转向积分项和是否有未清除的命令。

## 5. 常见模式与处置建议

| 观察到的模式 | 常见原因 | 建议 |
| --- | --- | --- |
| `pitch_deg` 远离目标且电机输出方向使误差更大 | 电机方向、姿态轴或符号配置错误 | 悬空低功率验证方向；不要先提高 Kp。 |
| P/D 项频繁大幅反号，电机快速抖动 | Kp/Kd 过大、机械松动、IMU 振动 | 降低增益，固定 IMU 和线束，检查轮胎与减速箱间隙。 |
| I 项持续单向累积或长期主导输出 | Trim 不准、机械重心偏、积分过强 | 先校准 Trim/重心，再谨慎启用 Ki。 |
| 平均轮速明显跟不上目标，俯角偏置已接近限幅 | 电机能力不足、低电压、地面阻力或速度 Kp 不足 | 查电池和机械，再小步调速度 Kp。 |
| 左右轮同向但差速度误差长期不收敛 | 编码器比例/方向不一致、转向方向错误、转向输出限幅 | 对照左右轮速度和 `applied_turn_motor_command`，先确认符号。 |
| 请求转向很大，但实际转向输出明显较小 | 平衡环占满混控余量 | 降低速度、转向目标或平衡输出需求；不要盲目增大 `max_turn`。 |
| 航向持续漂移但 `gyro_z_dps` 接近零 | 轮距、轮径、编码器比例不准，或轮胎打滑 | 以已知角度慢速转弯校准轮距；检查轮胎打滑。 |
| `accelerometer_pitch_deg` 瞬时偏离很大而 `pitch_deg` 平稳 | 加速、刹车、地面冲击 | 这是加速度计受线加速度影响的正常现象，结合陀螺仪判断。 |

## 6. 交接时应附带的信息

除 CSV 外，请一并交付：固件提交版本、`vehicle_config.h`、电池电压/载荷/地面条件、测试动作的时间点、是否连接 Wi-Fi 上位机，以及故障前后的串口 `L,...` 日志。分析结论必须写明使用的 CSV 时间区间、参数值和可复现实验步骤。

> 安全提示：参数调试先轮胎悬空验证方向，再以低速地面测试。保持可立即断电或发送停止平衡命令的条件；不要在人员或障碍物附近做首次后退、转向或高输出测试。
