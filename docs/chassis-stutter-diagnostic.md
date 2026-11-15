# 持续前推走走停停的诊断

现场已确认：摩擦轮、底盘跟随和小陀螺均关闭仍顿挫；两板均已Programming Done / Verify OK。上板DAP烧录问题已恢复，不能再用未烧录成功解释本次现象。

尚未取得实车停顿时的输入/轮速/电流时间序列，未确认根因。本版新增持久诊断，不改变底盘增益、速度、加速度、100ms超时阈值或使能逻辑；没有通过延长断流时间消除停机现象。

后续更新：用户截图已确认有效输入超时94次、最大间隔410ms，且两板始终Run。上板发送端已针对单次发送丢帧和最后邮箱争用修正，当前固件与验证说明见`can1-chassis-input-fix.md`；本页保留诊断版字段和操作说明，LOWER固件未因上板发送修正改变。

## 按代码可以区分的路径

1. 下板`board_prepare_chassis_input()`发现有效输入已超100ms，或上板传来的使能标志为0，则四轮立刻发零，PID及缓变目标清零。恢复后重新缓加速。连续发生会表现为反复起步。
2. 单轮反馈从未出现或超过100ms，该轮归零，其他轮仍闭环。即使不到100ms，反馈间隔异常也可能影响闭环，不能只用超时次数排除。
3. 持续非零前进请求本身不会进入正常停车分支。若现场出现零运动请求，则需要查实际ch0/ch1、key.sw及VTM/遥控是否交替覆盖，不能只根据手上仍推杆就判定软件输入始终非零。
4. 目标连续且反馈及时，输出却频繁正负饱和，应检查轮向/反馈符号、实际反馈时序及速度环参数。当前Kp=0.95，按现有尺度约100.5rpm误差即可达到±8192输出上限；这说明灵敏度高，并非没有实测也能认定参数必然不稳定。
5. 输出稳定非零但电机间歇停转，继续查电调保护、供电与机械阻滞。软件“发送入队成功”不等于电调持续接受并执行。

## 最短现场操作

本版只需更新`firmware/LOWER.hex`。下板Watch添加并展开`lower_diag`，确认`diagnostic_version=1`。上板无需为此版本更新。

两板均保持Run，摩擦轮/跟随/陀螺关闭，底盘先架空。先记录下面字段的基线，再保持固定前推约5秒，观察顿挫期间哪些计数增长；松杆后继续在Run状态读取。不要在试验中暂停上板或下板，否则会人为制造断流/调度间断。

| Watch字段 | 判断方法 |
|---|---|
| `lower_diag.input_loss_count` | 增长代表下板实际经历了输入使能从有效变无效；恢复不会清零 |
| `lower_diag.last_input_loss_reason` | 2=上板发送禁用标志（按当前上板源码，应继续检查遥控/VTM输入更新时间）；3=下板有效CAN1输入超100ms；1=尚未收过有效输入，通常仅启动阶段出现 |
| `lower_diag.last_input_loss_age_ms`、`last_input_loss_tick_ms` | 最近一次输入掉使能时的板间输入年龄与发生时刻；恢复后保留 |
| `lower_diag.input_age_ms`、`input_max_gap_ms` | 当前输入年龄/有效板间操作帧最大间隔；不是原遥控无线帧的年龄 |
| `lower_diag.feedback_stale_count[4]` | 各轮从有效变无反馈的次数；初始尚未收反馈也可能记1次，要比较试验前后的增量 |
| `lower_diag.feedback_max_gap_ms[5]` | 四轮及yaw已收到反馈之间的最大间隔；恢复后仍保留峰值 |
| `lower_diag.wheel_saturation_count[4]` | 各轮达到输出上限的循环累计次数；启动时增长正常，结合固定目标下的增量判断 |
| `lower_diag.wheel_output_sign_flip_count[4]` | 计算电流指令正负切换次数；正常制动、换向也会增长，不能仅凭非零断定振荡 |
| `lower_diag.zero_request_count` | 有效输入期间，最终运动请求从非零转零的次数；松杆本身会增加1，持续前推阶段增长才需要查输入/模式 |
| `lower_diag.calibration_reset_count` | 收到校准清PID请求次数；持续前推时异常增长需核对校准标志 |
| `lower_diag.control_max_gap_ms` | 底盘任务最大循环间隔；调试暂停会污染该峰值 |
| `lower_diag.send_gate_drop_count` | 计算出非零后，发送封装再次检查使能并强制归零的次数 |
| `board_diag.can1.busoff_count`、`board_diag.can2.busoff_count` | 分别定位板间侧/底盘侧CAN是否发生Bus-Off；看增量 |
| `lower_diag.can2_tx_fail_count` | CAN2帧入队失败累计次数，包括yaw帧；增长需结合总线状态及反馈进一步检查 |

优先把`input_loss_count`、`last_input_loss_reason`、`feedback_stale_count`、`feedback_max_gap_ms`、`wheel_saturation_count`、`wheel_output_sign_flip_count`、`zero_request_count`及CAN1/CAN2 Bus-Off计数在试验前后的值一起提供。这样才能区分“软件切了电流”与“闭环来回制动”，避免凭体感继续改参数。

如果输入掉线计数增长，先修复相应输入/总线链路；如果这些计数不增长，但反馈间隔有明显尖峰，先解决反馈时序；只有输入与反馈均连续、目标也稳定时，才根据输出与轮速记录进一步调速度环。上述计数没有异常也不能排除较短的电调保护或机械问题。

## 验证和固件

运行实际ARM底盘任务，保持固定前推1秒，逐毫秒注入CAN2反馈、每10ms注入CAN1输入，检查四轮每轮输出连续、符号正确、没有自主进入停车。另验证输入100ms超时停机只记一次、恢复保留原因、上板禁用与板间超时原因不同、单轮反馈中断/恢复记录保留，以及输出正负变化计数。

该测试不模拟电机惯量/摩擦，不证明实车PID稳定，也不证明真实CAN链路正常。

回归结果：SINGLE 21、UPPER 4493、UPPER_GUARD 3502、LOWER 7263个断言通过。LOWER Keil编译0错误、0警告；测试替身编译仍有已有警告。没有代用户烧录或操纵实车。

本诊断轮次LOWER.hex SHA256：`9971C86FA5CB545B189BC9E9E6C721C6A1E0C28029CD62F66B0D4CBAA420EF2D`。当前LOWER已随后更新小陀螺行进，诊断字段保留，最新固件记录见`spin-moving.md`。

此前下板固件备份在忽略目录`MDK-ARM/build/stutter-diagnostic-backup/LOWER.hex`。
