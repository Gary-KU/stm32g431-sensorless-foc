# mymotor FOC 上位机 (host)

通过 **USART1 (PB6=TX / PB7=RX, 115200 8N1)** 控制与监视 STM32G431 无感 FOC 板。

## 启动

```
双击 host/run_gui.bat          # 或
python host/foc_gui.py COM15   # 指定串口
```

依赖：`pyserial` `matplotlib` `numpy` `tkinter`（本机均已安装）。

## 硬件接线（重要）

| 项 | 说明 |
| --- | --- |
| 调试 | DAPLink → SWD（PA13/PA14），用 `pyocd` |
| **串口** | **独立 USB-TTL 模块**（CP210x / CH340）→ PB6(TX) / PB7(RX)，**交叉接线**，GND 共地 |

> ⚠️ **不要用 DAPLink 的虚拟串口**。DAPLink 的 VCP 与调试通道共用同一个 USB 设备/固件，
> 持续遥测（50 Hz）会让它崩溃，**SWD 与串口会同时失效**。
> 已实测复现一次：拔插 USB 后恢复，MCU 本身无恙（FOC 的 `run_ms` 自主保护已自行停机）。

## 协议

### MCU → 上位机

```
$D,state,fault,spref,speed,id,iq,vd,vq,vbus\r\n        周期 50 Hz（可调）
$P,<参数名>,<值>\r\n                                   参数设置确认
```

| 字段 | 含义 |
| --- | --- |
| state | 0=IDLE 1=FAULT_STOP 2=BRAKSTOP 3=NORMSTOP 6=ALIGN 8=OL_START 9=CL_RUN 10=CTRL_RUN |
| fault | 0=无 1=过流 2=堵转/失步 |
| spref / speed | 参考 / 实际转速 (Hz) |
| id / iq | d/q 轴电流 (A) |
| vd / vq | d/q 轴电压 (V) |
| vbus | 母线电压 (V) |

### 上位机 → MCU（行尾 `\n` 或 `\r\n`）

| 指令 | 说明 |
| --- | --- |
| `RUN <hz>` | 设定转速并启动（负值=反转） |
| `R <hz>` | 同 RUN |
| `S <hz>` | 只改参考，不启动 |
| `STOP` | 受控减速停机 |
| `DIS` | 解除使能 |
| `P <名> <值>` | 设置运行参数 |
| `?` | 立即回一帧遥测 |

### 可设置参数

| 名 | 全局量 | 说明 |
| --- | --- | --- |
| `rs` `ls` | g_foc_rs_ohm / g_foc_ls_h | 定子电阻 / 电感 |
| `pllki` | g_foc_pll_ki | 观测器 PLL 积分增益 |
| `thetaoff` | g_foc_thetaoff_sf | 角度补偿系数 |
| `force` | g_foc_speed_force_hz | 闭环切换点 (Hz)，0=用编译默认(25) |
| `decel` | g_foc_stop_decel_hzps | 停机减速率 (Hz/s) |
| `octrip` `ocinst` | g_foc_oc_trip_a / oc_inst_a | 过流阈值 (A) |
| `stallms` | g_foc_stall_ms | 堵转判定时间 (ms)，0=关 |
| `fw` `fwgain` `fwmax` | 弱磁 使能/增益/上限 |
| `obs` `fluxwc` `fluxkp` `fluxki` `psi` | 磁链观测器 |
| `mtpa` `ld` `lq` | MTPA |
| `dt` `dtith` | 死区补偿 (µs) / 符号阈值 (A) |
| `period` | 遥测周期 (ms)，默认 20 |

## 界面

- **串口选择/连接**，状态栏实时显示 `state` / `fault`
- 转速输入框 + 滑条（0~120 Hz）
- ▶ 正转 / ◀ 反转 / ■ 停止 / 解除使能 / 清故障
- 快捷参数：闭环切换点、弱磁、MTPA、PLL Ki
- 三路实时曲线：转速（参考+实际）、d/q 电流、母线电压（约 12 s 窗口）
- 原始报文日志

## 命令行快速测试

```bash
python - <<'EOF'
import serial, time
s = serial.Serial('COM15', 115200, timeout=0.2); time.sleep(0.3)
s.write(b'RUN 40\r\n'); time.sleep(12)
s.write(b'?\r\n'); time.sleep(0.4)
print(s.read(4096).decode('ascii','replace')[-200:])
s.write(b'STOP\r\n')
EOF
```

## 启动时序（实测，20 V 母线）

```
0.0s  ALIGN       (2 s, Id=0.5A 定位)
2.0s  OL_START    I/F 强拖，4 Hz/s
8.5s  到达 25 Hz  →  切 CL_RUN（闭环）
9s    闭环加速到目标转速
     ...  稳态 ±0.1 Hz
run_ms(默认 30s) 到  →  受控减速  →  NSTOP
```

> 启动约 **9 秒到 40 Hz**。若要更快，提高 `USER_MOTOR1_ACCEL_START_Hzps`
> （当前 4 Hz/s，需确认转子仍能跟随）。
