# 视觉模块与主板 I2C 通信操作手册

## 1. 接线

相机 ESP32-S3 是 I2C 从机，地址为 `0x42`；主板 ESP32-S3 是 I2C 主机。

| 信号 | 相机模块 | 主板示例 | 说明 |
| --- | --- | --- | --- |
| SDA | GPIO47（扩展接口 IO47） | GPIO1 | 双向数据线 |
| SCL | GPIO48（扩展接口 IO48） | GPIO2 | 时钟线 |
| GND | GND | GND | 必须共地 |
| 电平 | 3.3 V | 3.3 V | 禁止接入 5 V I2C 电平 |

在 SDA、SCL 上只保留**一组**约 4.7 kΩ 上拉至 3.3 V 的电阻。主板 GPIO1/2 定义位于平级 `balance_car` 工程的 `include/config/board_pins.h`；若主板已占用它们，只修改该处的视觉 I2C 引脚定义。不要使用相机 GPIO4/5，它们已连接 GC2145 SCCB。

## 2. 读取规则

- 总线频率：400 kHz。
- 主板每 20 ms（50 Hz）直接从地址 `0x42` 读取 16 字节。
- 相机端始终保存最新视觉结果；主板读取不到新帧时会得到相同的 `sequence`，这不是通信故障。
- `CRC-8/ATM`：多项式 `0x07`，初值 `0x00`，覆盖前 15 字节。

## 3. 16 字节数据包

| 字节 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | `magic` | `uint8` | 固定 `0xA5` |
| 1 | `version` | `uint8` | 当前为 `1` |
| 2–3 | `sequence` | `uint16` LE | 每帧视觉结果递增 |
| 4 | `flags` | 位图 | bit0=`found`，bit1=`is_held`，bit2=`target_valid` |
| 5–6 | `center_error_permille` | `int16` LE | 中线相对图像中心，`-1000~1000` |
| 7–8 | `target_error_permille` | `int16` LE | 前瞻点相对图像中心，`-1000~1000` |
| 9–10 | `angle_cdeg` | `int16` LE | 局部航向角，单位 0.01° |
| 11 | `valid_rows` | `uint8` | 连续有效扫描行数 |
| 12 | `missed_frames` | `uint8` | 连续失线帧数 |
| 13 | `threshold_used` | `uint8` | 当前实际二值阈值 |
| 14 | `reserved` | `uint8` | 固定为 0 |
| 15 | `crc8` | `uint8` | 前 15 字节 CRC |