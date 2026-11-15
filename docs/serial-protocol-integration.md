# 上位机 AA55 协议联调

本功能在 `test/serial-protocol` 分支开发。底盘流畅行进及小陀螺行进的保存提交是 `70f229a`；随后合并远端新增的 `11.7.md`，基线为 `e859877`。

## 接口和编码

上位机连接**上板 USB CDC 虚拟串口**。USART1/3/6 分别继续服务原图传、遥控、裁判系统，不分配给此协议。串口工具可填 115200；USB CDC 不以此设置改变 USB 传输速率。下板不处理该协议。

帧为 `AA 55 CMD LEN DATA SUM8`，SUM8 从 CMD 累加至 DATA 最后一字节，取低 8 位。所有数值显式小端编码：float32、int32、bool 单字节 0/1，无结构体填充。旧 `AA ... FF` 格式不再支持。

| CMD | 请求 DATA | 响应 DATA |
|---|---|---|
| 01 | yaw(float32)、pitch(float32)、fire_status(uint8)，LEN=9 | 空，LEN=0 |
| 02 | 空，LEN=0 | health(int32)、fire_number(int32)、heat(float32)、ZoneIsOurs(uint8)、Stop(uint8)，LEN=14 |
| 03 | 空，LEN=0 | target(float32)、fire_status(uint8)，LEN=5 |
| 04 | stop(uint8)，LEN=1 | 无回传 |

用户已确认以下联调语义：

- yaw/pitch 是**度、相对当前控制目标的修正量**，正负号沿用原自瞄加角方向。每轴接受有限的 [-180,180]，机械俯仰限位仍由原云台代码执行。
- 保留原键鼠模式 `key.sw=1`、鼠标右键 `mouse_btn_r=1` 自瞄使能。收到 CMD01 不会自动切换遥控模式或绕过遥控在线、校准和急停门控。遥控器模式下不执行上位机自瞄或开火。
- 同一修正量只消费一次；控制任务消费前收到多条 CMD01 时保留最新一条，不累加旧目标。手动鼠标模式会消费并丢弃角度修正，切回自瞄不会补执行旧角度。
- 自动开火仅在最近一次 CMD01 收到后 200ms 内有效；需要持续发送，建议 20–50Hz。查询和错误帧不会续期。超时只清除上位机自瞄/开火请求，不影响正常手动行进。
- CMD02 health=裁判 `Robot_Status.HP`；fire_number=**剩余17mm弹量** `Robot_Remaining_Ammo.Booster_17mm`；heat=17mm当前裁判热量（转为 float32）。尚未收到相应裁判数据时数值默认0。
- ZoneIsOurs 的区域和裁判位映射尚未定义，**固定 false**，不能用于区域决策。
- Stop 仅按比赛阶段 `Game_Status.Stage_Enum != 4` 回传，与 CMD04 的锁存状态独立。未收到比赛阶段时默认 true；此字段不额外接管电机。裁判值为最近收到的缓存，当前不提供新鲜度字段。
- CMD03 target 是 IMU **实际测量**的 yaw 相对云台任务启动、500ms初始化等待后的首个有效姿态之差，单位度，范围 [-180,180]。不是控制目标角，也不是累计转圈数。USB重连或急停不会改变参考零点，重启控制板才会重新取参考。
- CMD03 fire_status 是当前发射任务通过模式、超时及使能门控后的**发射使能**，不是弹丸实际离膛检测；CMD04锁存时为false。它也反映手动发射使能。
- CMD04 true 锁存全车电机输出停止：上板本地pitch/发射门控为零，通过现有 CAN1 禁用标志停止下板底盘与yaw。沿用现有周期、邮箱取消和断流保护，不新增CAN报文或改变协议版本3。USB断开/重连不会解除锁存；仅 CMD04 false 或控制板重启解除。释放时清除旧自瞄和开火请求，需新 CMD01 才能再次自动开火；原遥控有效且拨杆/摇杆仍有输入时，手动控制会恢复。
- CMD01空应答表示合法帧已处理，不表示电机已执行：急停锁存或手动模式下，仍应答，但不执行自瞄。CMD04不应答，CMD02的Stop也不是急停回读字段。

## 已纠正的原文示例

原协议示例的 -3.75 编码和校验有误。yaw=12.5、pitch=-3.75、fire=true 的正确报文：

```text
AA 55 01 09 00 00 48 41 00 00 70 C0 01 C4
```

原文 `00 00 78 C0` 实际是 -3.875；保留这组错误数据时校验应为 CC，原文 C9 不成立。固件按正确累加和校验，拒绝原文错误示例。

常用报文：

```text
查询裁判：AA 55 02 00 02
查询云台：AA 55 03 00 03
急停置位：AA 55 04 01 01 06
急停解除：AA 55 04 01 00 05
零修正、不开火：AA 55 01 09 00 00 00 00 00 00 00 00 00 0A
CMD01应答：AA 55 01 00 01
```

## 接收和发送行为

USB中断将数据复制至512字节环形缓冲区；任务在缓冲区重新用于下一包之前取得独立数据。支持拆包、粘包、噪声、错误长度/命令/校验后的重同步；错误帧不产生应答、不续期、不修改急停。半帧间隔超过50ms丢弃；排队超过200ms的接收数据不执行，防止暂停调试后追执行旧指令。

缓冲区溢出或USB重连清空半帧、待发送回复和自瞄/开火请求，保留急停锁存。回复队列32帧；BUSY保留队首，下一次任务唤醒重试；队满丢弃新回复并计数，急停仍执行。主机应限制请求速率并及时读取应答。

CDC发送复制到独立持久缓冲区，发送完成前不覆盖；断开状态/空类句柄不解引用。`pc_diag.tx_sent` 是USB栈接受发送的次数，不代表上位机已接收。

## 联调步骤

1. 烧录本分支的UPPER固件；LOWER可以保留之前已成功验证的小陀螺行进版本。板间协议仍为3，接线不变。若使用本分支两板HEX，也需分别确认下载校验成功。
2. 先用上板USB虚拟串口查询CMD02/03，确认长度、校验和字段。无裁判时应看到health/ammo/heat=0、Stop=true、ZoneIsOurs=false。
3. 先发送不带开火的零角度CMD01，检查固定空应答；然后在原右键自瞄模式下发送小角度修正，验证正负号、机械限位和“一包只修正一次”。
4. 用CMD04 true检查底盘、yaw、pitch、摩擦轮及拨弹输出均为零；USB拔插后仍停止。CMD04 false解除后，旧自瞄/自动开火不会恢复；另行检查原遥控控制恢复。
5. 最后验证持续CMD01和超过200ms不发送时的自动开火使能，检查 `pc_diag` 与上下板既有诊断；不要用原文带错误校验的报文判断接线故障。

附带主机客户端（需要Python；实时串口需先安装pyserial）：

```powershell
python scripts/serial_protocol_client.py --self-test
python -m pip install pyserial
python scripts/serial_protocol_client.py --port COM5 status
python scripts/serial_protocol_client.py --port COM5 attitude
python scripts/serial_protocol_client.py --port COM5 aim --yaw 1 --pitch 0
python scripts/serial_protocol_client.py --port COM5 stop
python scripts/serial_protocol_client.py --port COM5 release
```

`aim`默认不开火。`--fire`显式申请自动开火；`--count 20 --interval 0.05`（放在子命令前）可连续发送。持续发送非零修正会持续改变目标，不能把同一非零相对修正当作绝对角反复发送。

Watch推荐：`pc_diag`、`usb`、`motion.auto_fire_detect`、`shooter_control.shoot_flag`、`gimbal.target_yaw`、`board_diag`和`lower_diag.motion_enabled`。

## 软件验证

`tests/run_protocol.py`在ARM/Unicorn中执行实际串口解析、接收环形队列、回复队列、自瞄消费和发射入口，并回归原底盘/板间控制。覆盖正确示例、所有分割点、逐字节输入、单比特损坏、非法bool/NaN/Infinity、帧内重同步、字段字节序、时钟回绕、超时、急停/解除/重连、接收复制/溢出/过期和回复队满。

另执行实际 `usbd_cdc_if.c` 适配层，模拟USB栈与任务通知，检查断连/空类句柄、无任务句柄时接收重挂、接收源缓冲覆盖、发送BUSY重试与独立持久缓冲。USB CDC的物理枚举、真实USB栈BUSY/完成时序、实际裁判输入、整车制动延迟和电机响应仍需上述实机联调，软件测试不代表这些项目已完成。

本次记录：SINGLE 2207、UPPER 6727、UPPER_GUARD 5736、LOWER 20615，共35285个断言通过。测试桩编译仍有原有警告；正式UPPER/LOWER Keil工程均为0错误、0警告。Python客户端 `--self-test` 通过。

固件SHA256：

```text
UPPER.hex 71B67F9A80E1DCC6645364889C2A0E76117AE3E9207AF85090F8F8D0E0FB77B2
LOWER.hex 3BE5CBACDE31F2A29BDABC65DB6A0AD4BDDC1B8A9E571A47C89B9501B473AD15
```
