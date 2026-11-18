# 持续前推顿挫：有效板间输入超时

## 已取得的现场证据

用户确认截图计数是在两板始终Run、未暂停或重新下载的持续前推过程中增长。下板诊断版本为1，截图读取如下：

后续现场反馈：用户已确认本轮上板更新后普通行进流畅；随后小陀螺行进更新见`spin-moving.md`，其UPPER HEX与本轮完全相同。

| 字段 | 截图值 | 含义 |
|---|---|---|
| `input_loss_count` | 0x5E = 94 | 曾94次从输入有效变为无效 |
| `last_input_loss_reason` | 3 | 最近一次为有效板间输入超过100ms未更新 |
| `last_input_loss_age_ms` | 0x65 = 101ms | 正好跨过既有100ms阈值 |
| `input_max_gap_ms` | 0x19A = 410ms | 有效操作帧间隔曾达到410ms，正常计划为10ms |
| `input_age_ms`、`input_reason` | 12ms、0 | 截图当时输入已经恢复，历史故障记录仍保留 |
| `control_max_gap_ms` | 1ms | 下板任务未记录到长时间调度间断 |
| `can2_tx_fail_count`、`can2_esr`、`stale_wheel_mask` | 0、0、0 | 截图当前局部电机链路未显示这些异常，不能据此排除所有历史CAN2故障 |

这证明了一个直接致顿挫路径：下板有效输入超时后四轮归零，清PID及缓变目标；输入恢复后又从零加速。反复发生就会走走停停。它并不单独证明410ms间隔源于物理掉线、仲裁丢帧、上板任务延迟还是帧拒收。

## 找到并修复的上板发送缺陷

1. **上板CAN1原为单次发送。** `AutoRetransmission=DISABLE`使NART置1。依据[ST RM0090，bxCAN Nonautomatic retransmission mode](https://www.st.com.cn/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=1087)，第一次尝试即使只是仲裁失败，硬件也不会重发。CAN1上电机报文的较小ID比0x320优先，发送入队成功不能代表操作帧已送达。现在仅UPPER CAN1开启硬件自动重发；CAN2和SINGLE/LOWER原控制器设置不变。
2. **yaw先占用最后一个邮箱。** 原上板任务在同一轮先尝试2ms yaw帧，再尝试10ms操作帧。测试模拟仅剩一个空邮箱、两种消息同时到期，旧代码明确丢掉底盘入队机会。现改为到期0x320先入队，yaw仍保持原2ms周期和独立序号/超时。
3. **重发必须限制旧指令存活。** 新增UPPER CAN1三邮箱跟踪。正常1ms服务或再次发送时，仍未完成且已待发10ms的帧申请取消；取消异步完成前暂停新CAN1帧入队，确认TME释放后才接受新生成指令。Bus-Off恢复仍由原状态机统一取消旧请求，期限服务在恢复期间不重复干预。超过期限不是硬件自动失效：若CPU/任务被暂停，软件无法保证绝对10ms期限，仍需看任务最大间隔。

这些改动修复可复现的软件发送风险，但没有旧版逐帧ALST/TERR记录，不能把现场94次超时全都归因于仲裁或邮箱争用。摩擦轮关闭后仍有CAN电机反馈及零电流命令，因此关闭摩擦轮运动并不等于减少所有高优先级CAN通信。

底盘PID、加速度、速度上限、100ms输入断流保护和协议3不变。未通过延长100ms阈值掩盖断流。上板yaw/pitch/发射增益及指令来源不变，CAN1发送的底层重试和过期待发帧处理会应用于该总线上板发送的所有帧。

## 固件与验证

- 本次仅需更新`firmware/UPPER.hex`。LOWER保留当前诊断版；重新编译LOWER后的HEX与现有固件SHA256完全相同。
- UPPER.hex SHA256：`7CC71A339BEF4635EE9A10AEC30CE041EA96BD69A5093C7BA177C200C6BE8A09`。
- 旧上板固件备份：`MDK-ARM/build/can1-retry-backup/UPPER.hex`。
- 回归：SINGLE 21、UPPER 4533、UPPER_GUARD 3542、LOWER 7263个断言通过；上下板Keil均0错误、0警告。外设替身测试编译仍有已有警告及测试专用未使用变量警告。
- 覆盖最后一个邮箱争用，三邮箱逐一过期、计时回绕、取消失败重试、取消未完成禁发、新指令恢复、已完成请求不误取消。测试模型不验证CAN比特级仲裁、实际ACK和真实负载时序。
- 未代用户烧录或操纵实车，尚未完成修复后的实车验证。

## 更新后观察

烧录上板本身会使下板超时一次，因此两板恢复Run并稳定后先记下新的`lower_diag.input_loss_count`基线；固定前推时该值应不再持续增加。不要仅因累计数仍为94/95就认定修复失败，也不要在试验中暂停任一板。

若仍增长，上板展开`board_diag`：

| 新增字段 | 解释 |
|---|---|
| `chassis_tx_fail_count` | 操作帧未能入队的次数，包括取消未完成/邮箱忙/控制器不可发送 |
| `chassis_tx_max_gap_ms` | 操作帧成功入队的最大间隔，不代表接收间隔或ACK |
| `poll_max_gap_ms` | 上板板间任务最大执行间隔，判断是否存在任务/调试间断 |
| `can1_tx_expired_count` | 10ms后仍未完成而成功申请取消的帧数 |
| `can1_tx_abort_fail_count` | 申请取消失败次数 |
| `can1_mcr`、`can1_tsr` | 控制/发送状态快照；UPPER CAN1的NART位0x10应为0 |

再结合既有`board_diag.can1.busoff_count/stage/last_error_esr`及下板`board_diag.chassis_frame_count`、`board_chassis_rx_count`、`chassis_reject_reason`区分总线错误、上板没能发送、以及下板接收后拒帧。

截图底部`feedback_stale_count[4]`无法求值是Watch表达式越界：四轮数组合法索引是0、1、2、3。输入`lower_diag.feedback_stale_count`后展开即可；先前文档表格中的`[4]`表示数组长度，不是第五个元素。
