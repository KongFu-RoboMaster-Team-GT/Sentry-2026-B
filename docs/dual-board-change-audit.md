# 双板改动审查（2026-10-31）

本报告记录10月31日审查。2026-11-14按用户要求关闭yaw超速/跟踪误差锁停，并调整yaw响应参数；当前参数及重新启用开关见README的yaw说明。

基准为仓库提交 `63f8ab1`。检查全部已跟踪差异，以及新增双板代码、配置、Keil工程、脚本和测试。此提交不是每次实车测试的固件快照，无法据此确定故障首次出现在哪次烧录。

结论：新增路径中确有遗漏，已修复；尚未确认“摩擦轮/拨弹启动就CAN1 Bus-Off”的源码根因。编译通过、仿真通过、恢复计数增长均不等于实车问题已解决。

## 确认的问题及本次修复

1. **yaw数据异常连带停止pitch。** 新增的合并有效性检查会因yaw角/速度无效同时发送pitch零输出。已分轴检查；pitch参数、方向、映射、限幅不变。失去控制输入/校准仍共同停机。新增测试先在修复前失败，再验证修复通过。
2. **校准路径漏发射击停机请求。** 新增提前返回跳过原`CAN_cmd_shoot(0,0,0)`。已恢复，并在上板射击CAN封装中限制校准期间为零，避免下个射击任务周期覆盖停机请求。原射击算法和拨杆含义不变。
3. **下板取消INS任务，却保留向空任务句柄发通知的中断。** 下板不再启用相关IMU/PG0中断，回调跳过下板INS；其他角色通知前检查句柄，每次初始化ISR唤醒标志。未证实现场触发，不用于解释上板Bus-Off。
4. **单板误用双板缓变专用停止条件。** 将`stop_requested`限定为LOWER，恢复SINGLE原小输入停止判据；不改变双板底盘参数。

## 原版内容核对

通过`git diff --exit-code HEAD -- ...`确认以下文件无差异：

- `Task/shooter_task.c/h`：摩擦轮/拨弹PID、转速、连续/单次联动和周期。
- `User/Algorithm/alg_pid.c/h`、`User/Algorithm/SMC.c/h`、`User/Heat/heat_control.c`：算法实现。
- `User/remote_control.h`、`Core/Src/can.c`：遥控结构、CAN位时序及自动恢复/重发初始设置。

`remote_control.c`只添加条件包含及成功解码后的时间戳，不改通道解包、阈值、模式映射。
pitch的SMC参数、反馈符号、摇杆映射、角度约束、输出上限/死区/斜率与基准一致。本次修改异常数据门控，没有重新调pitch参数。

## 全部改动范围

| 文件/范围 | 变化及审查结果 |
|---|---|
| `Boards/bsp_can.c` | 清零滤波结构、首次配置即设置分界14，修复原未初始化字段。 |
| `Core/Inc/main.h`、`User/board_config.h` | 引入角色、周期/超时/协议；默认SINGLE，上板1，下板2。 |
| `Core/Src/main.c`、`Core/Src/freertos.c` | 上下板任务分工，下板跳过接收机/IMU初始化，新增板间任务。 |
| `Core/Src/gpio.c`、`Task/ins_task.c` | 本次修复下板INS中断遗留路径及通知句柄检查。 |
| `Task/control_task.c`、`User/remote_control.c` | 原合法输入更新时间戳，不重解码遥控器。 |
| `Task/chassis_task.c/h` | 下板用板间输入驱动原1ms本地PID，枪口方向变换、前推通道修正、目标缓变及断流清积分；增益/输出尺度未提高。 |
| `Task/gimbal_task.c/h`、`User/yaw_control.h` | yaw解缠绕、目标历史同步、慢速手动/松杆保持、输出限制、故障保护与方向诊断；本次修复两轴门控和校准停射。 |
| `User/control_config.h`、`User/control_geometry.h` | yaw临时调试参数、零点3156、坐标/缓变工具；尚未完成yaw实车调参，编码器方向仍需实测。 |
| `User/Middleware/drv_CAN_receive.c/h` | 角色分配发送权、接收校验、断流停机和诊断；双板采用局部报文头及短临界区，改善原共用报文头的抢占窗口。本次恢复校准停射意图。 |
| `User/board_link.c/h` | 协议、序号/校验、超时、反馈转发、非阻塞CAN恢复及诊断。 |
| `MDK-ARM/dual_board.uvprojx`、`scripts/*.ps1` | 角色宏、独立输出及构建脚本；构建不烧录。 |
| `MDK-ARM/FINAL-2026.uvprojx` | 已有本地Flash DLL差异保留，不回退用户调试器设置。 |
| `Middlewares/Third_Party/FreeRTOS/Source/tasks.c` | 已有一处空白差异，无逻辑变化，保留。 |
| `.gitignore`、`README.md`、本报告 | 构建忽略规则、固件保留及说明。 |
| `MDK-ARM/EventRecorderStub.scvd` | 空事件的调试器描述，不参与控制。 |
| `tests/` | 实际C代码回归及外设替身；本次补真实射击任务、热控实现集成验证。 |
| `firmware/UPPER.hex`、`firmware/LOWER.hex` | 修复后的构建输出，未代用户烧录或实车验证。 |

## CAN审查及剩余限制

| 总线 | 电机命令唯一发送者 | 命令 |
|---|---|---|
| 共享CAN1 | 上板 | `0x200`一/二槽摩擦轮、第三槽拨弹；`0x1FE`第二槽pitch。 |
| 下板CAN2 | 下板 | `0x200`四槽底盘；`0x1FE`第一槽yaw。 |

未发现下板向共享CAN1重复发送电机命令。两条独立物理总线各有`0x200`不构成冲突。

正常新增板间流量：上板底盘输入100Hz、yaw命令500Hz，下板状态50Hz、有效yaw反馈50Hz，共700帧/秒。射击关闭时原任务也每毫秒发送零电流，拨开并未新增一路1kHz发送。不能据此认定没有负载问题，也不能认定已过载。

**发送可靠性仍有限制：** 沿用原`AutoRetransmission=DISABLE`，HAL入邮箱成功不保证最终发送成功，仲裁丢失/报文错误不自动重发。邮箱满则丢弃本次请求，下次任务重新生成。新增发送者增加竞争，可能造成板间断流；尚无持续饥饿证据，正常仲裁也不能解释TEC增长至Bus-Off。本次不做未经验证的重发策略、频率或波特率调整。

`last_error_esr=0x00FD0057`中BOFF/EPVF/EWGF置位，最后错误LEC=5。TEC可见字段只是九位计数器的低八位，不能将253当作完整计数。此值证实位错误和离线，不能确定具体接线、终端、收发器、供电、时序或节点原因。恢复状态机目前未发现确定的死锁或共享滤波复位问题；恢复成功不代表触发原因消失。

## 验证

- ARM测试：SINGLE 21、UPPER 3494、LOWER 1063，共4578项断言通过。
- 运行真实射击任务、PID、热控和CAN封装，覆盖连续/单次联动、关闭、校准停机、穿插pitch命令后继续发送正确射击帧。裁判输入是替身。
- NaN yaw角/速度、NaN pitch角/速度分别注入，验证只停止对应轴，有效轴继续响应，反馈恢复后输出为有限数。
- Keil UPPER/LOWER均0错误、0警告。测试编译仍有原源码缺末尾换行等警告，不与固件构建混淆。
- 本次编辑文件`diff --check`通过；全仓检查仍报告已有FreeRTOS空白差异，未改该用户文件。
- 替身不模拟实际收发器、电源、负载、仲裁和波形；未证明实车Bus-Off已解决，未完成yaw调参。本次修复分别涉及上、下板，需要更新两份固件。

参考：[ST RM0090](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-407-415-417-437-455-469-application-note-stmicroelectronics.pdf)的发送、错误管理和CAN_ESR说明。
