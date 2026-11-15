# 上板 USB 无设备的挂起路径修复

现场：上板已供电、使用新协议固件，确认换数据线、直连电脑及复位后仍无插入反应。Windows 当前未枚举到 STM32；用户在 Watch 确认：`hpcd_USB_OTG_FS.State=HAL_PCD_STATE_READY`，`hUsbDeviceFS.dev_state=0x04`。

这两个状态属于不同枚举：PCD READY=1，表示底层初始化完成；USB设备库4表示SUSPENDED，并不证明曾成功枚举。只有PCD自身的4才表示TIMEOUT。

## 已确认的软件路径

`USB_DEVICE/Target/usbd_conf.c` 配置 `low_power_enable=DISABLE`，但原挂起回调无条件执行 `__HAL_PCD_GATE_PHYCLOCK`，设置PCGCCTL的STPPCLK/STOPCLK位；原恢复回调没有清除该位，工程也未配置独立的USB唤醒EXTI处理。该行为来自原基线，与AA55命令的解析无关。

本次修改仅在 `test/serial-protocol` 分支：非低功耗模式下，挂起保持PHY时钟；恢复回调显式解门控。保留USB库的挂起/恢复状态处理，不伪造配置成功状态。正常底盘、yaw、pitch、射击控制和CAN调度不改。

[ST RM0090，OTG_FS_PCGCCTL寄存器](https://www.st.com/resource/en/reference_manual/rm0090-stm32f407-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) 说明，应用在USB恢复或新会话开始时应清除停止PHY时钟的位。本次修复补齐此路径，但**READY+SUSPENDED本身不足以确认现场故障根因**，仍需烧录实测。

## 实机验证

仅需重新下载UPPER并确认Programming Done/Verify OK；下板可保持原版本。主电源正常、程序Run，USB插好后复位，再验证冷启动和运行中拔插。

- Windows应出现STM32/USB串行设备和COM口；固件描述符为VID=0483、PID=5740。
- `hUsbDeviceFS.dev_state=3` 表示已配置，可继续AA55查询测试。
- 如果仍为4，再读取 `hUsbDeviceFS.dev_old_state`：1表示挂起前为默认状态，2表示已分配地址，3表示曾配置成功。也可能在调试暂停期间挂起，因此读取状态时需说明程序是否保持Run。
- Watch可只读查看 `*(volatile unsigned int *)0x50000E00`（USB FS PCGCCTL）。本配置修复后STOPCLK位bit0应为0；PHYSUSP等其他位需分别判断，不能要求整个寄存器一定为0。
- 若PHY时钟已运行但仍完全无连接事件，应继续检查USB连接器、D+/D-线路、收发器供电和软断开寄存器；不能仅凭当前状态认定接口损坏。

## 软件验证与边界

UPPER/LOWER Keil编译均0错误、0警告。既有协议/控制回归共35285个断言通过（测试桩仍有原有编译警告）。

新增 `tests/run_usb_power.py` 从正式UPPER.axf及DWARF读取真实函数、变量地址和成员布局，在Unicorn中执行实际挂起/恢复机器码。181项检查覆盖默认/已寻址/已配置状态的重复挂起恢复、旧停钟状态清除、正常模式不进入STOP，以及原低功耗可选分支的行为。

```powershell
python tests/run_usb_power.py
```

寄存器在测试中为模拟内存，未仿真真实PHY、USB线上的枚举过程、Windows驱动绑定或STOP后的系统时钟恢复。当前工程关闭低功耗；本次不宣称支持完整STOP模式。实车验证尚待用户执行。

修复版SHA256：

```text
UPPER.hex C6F70CC2C9B7788F8C7A4CD77FF1C2AA155D0BAE163D6AC7DAEC47A93B891382
LOWER.hex 6C6505DA0081FF397197B6482BCA712AB78DF49E7E899480343FA4E9EE90F762
```
