# 机械、编码器与 IMU 调试上位机

入口为 `tool/vehicle_characterizer.py`。它沿用主板 Wi-Fi UDP 调试链路，但不参与平衡控制，也不会在线改写 PID 或方向配置。

```powershell
.\.venv\Scripts\python.exe tool\vehicle_characterizer.py
```

先烧录本工程当前固件，再让电脑连接主板热点。在上位机中使用默认的 `192.168.4.1`、本地 UDP `9000`、命令端口 `9001`，点击“连接并订阅”。连接正常时顶部会同时显示：

- `T,10`：既有姿态、轮速和控制遥测；
- `D,1`：新增的原始累计 tick、40 ms tick 增量和 tick/s。

原有 `tool/host_pyqt.py` 与 `tool/speed_turn_tuner.py` 继续使用 `T,10`，可忽略 `D,1`，因此不会因本工具失去兼容性。

## 安全边界

“电机 / 编码器方向”和“PWM / 死区”页只能在车轮悬空时使用。主板只在 `STANDBY` 或 `MANUAL_TEST` 接受 `MOTOR` 命令；每一侧最大为 `±0.35`，且上位机停止续发超过 1 秒会由主板自动关闭电机。点击“停止测试 / 断电”、取消扫描、断开连接均会发送 `STOP`。

IMU 零偏标定只在 `STANDBY` 接受，约持续 1 秒。标定期间不要触碰车体，也不要让电机转动。

## 1. 电机、编码器方向

1. 勾选“车轮悬空”，以 `0.10–0.15` 占空比依次点按左/右轮正反转、双轮正反转。
2. 人工观察并勾选：左右正向命令均让车前进；前进时两侧 signed tick / 轮速同号且为正。
3. 点击“Δv>0：右轮更快”，确认右轮比左轮快。`D,1` 状态会同时显示原始 tick、tick 增量和由当前轮径/CPR 换算的速度。
4. 用低风险、零速度的平衡测试分别确认 `motorOutputInverted`、速度环和转差环 `outputInverted`；勾选后生成冻结记录。

方向出错时先修改并验证 `include/config/vehicle_config.h` 中对应的 `leftDirectionInverted`、`rightDirectionInverted`、`motorOutputInverted`、速度/转差 `outputInverted`。确认后冻结；不要增加 PID 增益或积分去补偿符号错误。

## 2. 编码器比例

1. 在直线标尺上标记至少 2 m，设置真实距离、当前 `countsPerWheelRevolution`（默认 466）和轮径（默认 0.064 m）。
2. 起点静止时点击“记录起点 tick”；沿直线走至终点，点击“记录终点并计算”。
3. 前进和后退各做一次。表格将列出 signed Δtick、tick/m、固定 CPR 时的有效轮径，以及固定轮径时左右建议 CPR。
4. 查看汇总的左右比例系数。若差异明显，优先使用左右独立的编码器比例/比例系数，而不是把误差交给速度 PI。

默认值下每 tick 距离约为 `π × 0.064 / 466 = 0.000432 m`；40 ms 窗口内 1 tick 约为 `0.0108 m/s`。因此 `0.03 m/s` 以下的瞬时速度主要是量化噪声。本页直接计算累计 tick，不使用瞬时速度。

## 3. PWM 与死区

1. 保持车轮悬空，先确认电机驱动芯片允许 20 kHz PWM，再勾选允许扫描。
2. 左轮、右轮各扫描一次，建议 `0.04 → 0.30`、步长 `0.03`、每级 `1.5 s`；正反转各做一组。
3. 表格记录每一级稳态段的左右速度，摘要给出最小可靠起转 duty、近似斜率和线性外推死区。
4. 同时检查驱动器/电机温升和正反转切换冲击；发现异常立即停止。

低于 `0.03 m/s` 的点会被视为量化噪声区。若左右曲线有明显差异，后续控制应加入“每侧死区补偿 + 速度前馈”；PWM 频率与位宽不是常规 PID 扫描参数。

## 4. IMU 与姿态参数

1. 在 `STANDBY` 静止状态勾选确认，点击“执行陀螺仪零偏标定”。
2. 将车体置于已知机械角度，在“当前 accel offset”填入现有 `accelerometerAngleOffsetDegrees`，填写已知角并记录静态样本。工具会给出候选新偏置：

   `newOffset = currentOffset + knownMechanicalAngle - median(accelerometerPitch)`

3. 缓慢前后倾车，确认 `pitch` 和 `pitchRate` 都符合物理方向；若不一致，先检查 `pitchAxis`、`pitchAngleInverted`、`pitchGyroInverted`。
4. 在安全的零速度平衡状态记录一个窗口。目标是无持续漂移且内环原始输出平均值较小；每次仅小幅改变 `targetPitchDegrees` 后复测。
5. 互补滤波时间常数默认保持 `0.25 s`。只有加速时俯仰误差明显才考虑增大；不要为了看起来“更快”盲目减小。

## 记录与协议

每次连接自动在 `tool/records/` 写入遥测 CSV 和原始 tick CSV；“导出完整调试报告”额外生成包含方向确认、比例试验、PWM 曲线和 IMU 结论的文本报告。`tool/records/` 是实验产物，不应作为固件配置真值；确认后的值仍应明确回填至 `include/config/vehicle_config.h` 并提交版本控制。

新增的 Wi-Fi 命令仅用于此工具：

```text
C,<seq>,MOTOR,<left duty>,<right duty>
C,<seq>,IMU_CAL
```

两者均通过主板安全状态机处理，不能在平衡状态绕过保护。
