# Sensorless FOC for STM32G431 — BLDC Motor Control

[![MCU](https://img.shields.io/badge/MCU-STM32G431CBT6-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32g431cb.html)
[![FOC](https://img.shields.io/badge/control-Sensorless%20FOC-green)]()
[![IDE](https://img.shields.io/badge/IDE-Keil%20MDK--ARM-orange)]()
[![License](https://img.shields.io/badge/license-MIT-lightgrey)](LICENSE)

A complete, from-scratch **sensorless field-oriented control** implementation for a
JLC (嘉立创) STM32G431 FOC breakout board, with an eSMO observer, field weakening,
a serial host protocol and a Python GUI.

> **English**: this README is written in Chinese (the firmware comments are also in
> Chinese). A short English overview is in [English overview](#english-overview) at
> the bottom. The deep-dive bring-up log is in [`docs/FOC_BRINGUP.md`](docs/FOC_BRINGUP.md).

---

## 目录

- [特性](#特性)
- [硬件](#硬件)
- [控制流程](#控制流程)
- [实测性能](#实测性能)
- [电机参数（实测辨识）](#电机参数实测辨识)
- [仓库结构](#仓库结构)
- [编译与烧写](#编译与烧写)
- [上位机使用](#上位机使用)
- [串口协议](#串口协议)
- [文档](#文档)
- [已知问题 / TODO](#已知问题--todo)
- [许可与致谢](#许可与致谢)

---

## 特性

| 功能 | 说明 |
| --- | --- |
| **无感 FOC** | 定位 → I/F 开环强拖 → eSMO 观测器 → PLL → 速度环闭环 |
| **两种观测器** | eSMO（滑模，默认）/ 定子磁链（`g_foc_obs_mode`，实验性）|
| **弱磁 (FW)** | 默认**关闭**（可 `P fw 1` 开启）—— 20 V 母线基础电压极限 ~105 Hz，开启后约 110 Hz 但会引入 4 Hz 振荡 |
| **MTPA** | 凸极电机最大转矩电流比 |
| **保护** | 过流（2.5 A / 瞬时 4 A）、堵转/失步（400 ms）、主循环看门狗、五大 CPU 异常全部接入安全刹车 |
| **受控减速停机** | 用电流环限流减速，避免高速短接的大电流冲击 |
| **故障停机策略** | 转速 > 15 Hz 时用受控减速，低速才下管短接 |
| **串口上位机** | USART1 (PB6/PB7) + Python GUI，50 Hz 遥测 + 22 个在线可调参数 |
| **调试基础设施** | DWT 周期计数（ISR 负载）、黑匣子日志、寄存器级在线调参 |
| **主机单元测试** | `tests/` 下可编译运行的 FOC 数学库测试 |

---

## 硬件

**开发板**：嘉立创 STM32G431 FOC 开发板

| 项目 | 型号 / 引脚 | 备注 |
| --- | --- | --- |
| MCU | STM32G431CBT6 (Cortex-M4F) | 150 MHz（HSI PLL，未用 170 MHz boost）|
| 门驱 | **EG2133** | 单端 PWM 输入；无法滑行停车（下管短接制动）|
| 功率级 | 3 相半桥 | MOS 额定 20 A |
| 电流采样 | 3× 10 mΩ 低侧采样 + **SGM8634**（16.5×）| 满量程 ±20 A，0.004883 A/count |
| PWM | TIM1 中心对齐 **16 kHz** | CH1=PA8=W，CH2=PA9=V，CH3=PA10=U |
| 电流 ADC | ADC1 规则组（TIM1 更新中断软触发）| **U=PB14/IN5，V=PB11/IN14**，W=PB12/IN11 |
| 母线 ADC | ADC2 / PB15 | 分压比 36.3 V |
| 急停输入 | PB10 | |
| 串口 | **USART1：PB6=TX，PB7=RX**，115200 8N1 | PA9/PA10 被 PWM 占用 |

> ⚠️ **本板 W 相电流采样硬件故障**（运放输出异常），固件用 `Ic = -(Ia+Ib)` 重构。
> 详见 [`docs/FOC_BRINGUP.md`](docs/FOC_BRINGUP.md) §2。

---

## 控制流程

```
        ┌──────────┐    ┌───────────┐    ┌──────────┐    ┌──────────┐
 复位 → │  ALIGN   │ →  │ OL_START  │ →  │ CL_RUN   │ →  │ CTRL_RUN │ → 正常/故障停机
        │ 2 s 定位 │    │ I/F 强拖  │    │ 观测器接管│    │ 速度环闭环│
        └──────────┘    └───────────┘    └──────────┘    └──────────┘
          Id = 0.5 A      Iq = 0.5 A      切 θ 到 PLL     PI 速度环
          θ = 0           4 Hz/s 斜坡      33 Hz 切换点    有/无弱磁
```

**16 kHz 中断**：采电流 → Clarke/Park → 观测器(eSMO) + PLL → 速度环(1.6 kHz) →
电流环(d/q PI) → 死区补偿 → InvPark → SVPWM → 写 TIM1。

**状态机**：`0=IDLE 1=FAULT_STOP 2=BRAKSTOP 3=NORMSTOP 6=ALIGN 8=OL_START 9=CL_RUN 10=CTRL_RUN`

---

## 实测性能

20 V 母线，空载：

| 目标 | 结果 | 转速标准差 | 主导波动 |
| --- | --- | --- | --- |
| 35 Hz | ✅ 稳定 | **0.42 %** | 38 Hz（ωe，幅值 0.08）|
| 60 Hz | ✅ 稳定 | **0.22 %** | 65 Hz（幅值 0.05）|
| 80 Hz | ✅ 稳定 | **0.84 %** | 87 Hz（幅值 0.10）|
| 100 Hz | ✅ 稳定 | **0.77 %** | 6.8 Hz（幅值 0.05）|
| 110 Hz | ⚠️ 需开弱磁 | — | 20 V 母线电压极限 ~105 Hz |

- **可靠闭环区间**：**33 – 105 Hz**（电频率，弱磁关闭）；转速 < 33 Hz 时观测器不可靠
- **CPU 占用**：ISR 约 3400–4000 cycle ≈ **37–43 %**（预算 9375 cycle @ 16 kHz / 150 MHz）
- **启动时间**：约 10 s 到 40 Hz（2 s 定位 + 8 s I/F 斜坡）

> **关于弱磁**：`g_foc_fw_enable` **默认关闭**。实测开启弱磁会让整个转速范围的
> 标准差从 0.2–0.8 % 恶化到 3–4.6 %（弱磁积分器与速度环构成阻尼不足的回路）。
> 而 20 V 母线的电压极限基础转速已经是 ~105 Hz，弱磁只能多换约 5 %。
> 需要更高转速时用 `P fw 1` 打开，但需接受纹波。详见
> [`docs/FOC_BRINGUP.md`](docs/FOC_BRINGUP.md) §9.3。

---

## 电机参数（实测辨识）

| 参数 | 值 | 方法 |
| --- | --- | --- |
| 定子电阻 Rs | **1.25 Ω** | 定位阶段 `Vd/Id` |
| d 轴电感 Ld | **1.56 mH** | HF 注入 / 静态扫描 |
| q 轴电感 Lq | **2.39 mH** | 同上 |
| 凸极率 Lq/Ld | **1.53** | 真凸极（IPM 特性）|
| 转子磁链 ψr | **0.0175 Wb** | 反电动势标定 |

---

## 仓库结构

```
.
├── Core/                     应用层（CubeMX 生成 + 手写）
│   ├── Inc/
│   │   ├── foc_hw.h          硬件抽象：PWM/ADC/三角函数
│   │   ├── foc_math.h        Clarke/Park/SVPWM/角度运算（含主机测试桩）
│   │   ├── foc_serial.h      上位机串口协议
│   │   ├── motor_foc.h       状态机与运行时可调全局量
│   │   ├── user_motor.h      eSMO / 磁链观测器 / PLL / HFI
│   │   ├── user_mtr.h        电机参数（★ 主要调参入口）
│   │   └── user_vars.h       控制用数据结构
│   └── Src/                  对应实现
├── Drivers/                  STM32 HAL + CMSIS（未用的 DSP/NN/RTOS2 已移除）
├── MDK-ARM/                  Keil MDK 工程（Mymotor.uvprojx）
├── host/                     上位机（Python + tkinter + matplotlib）
│   ├── foc_gui.py
│   ├── run_gui.bat
│   └── README.md             协议与使用说明
├── tests/                    主机侧单元测试
│   └── test_foc_math.c
├── docs/
│   ├── FOC_BRINGUP.md        ★ 完整调试记录（踩坑、参数、数据、结论）
│   └── OPEN_LOOP_TEST.md     开环调试记录
├── Mymotor.ioc               STM32CubeMX 工程
└── README.md
```

---

## 编译与烧写

### 编译（Keil MDK-ARM）

需要 **Keil MDK-ARM V5** + **ARM Compiler V5 (armcc)** + **STM32G4xx DFP**。

```bash
# GUI
双击 MDK-ARM/Mymotor.uvprojx → Build

# 命令行
UV4.exe -j0 -b MDK-ARM/Mymotor.uvprojx -o build_log.txt
```

产物：`MDK-ARM/Mymotor/Mymotor.hex` / `.axf`

### 烧写（pyOCD + DAPLink）

```bash
pyocd flash --pack <Keil.STM32G4xx_DFP.x.x.x.pack> \
            -t STM32G431CBTx -e chip MDK-ARM/Mymotor/Mymotor.hex
```

> ⚠️ **重要坑**：`pyocd reset` 之后内核**不一定真正运行**，代码可能根本没跑。
> 烧写后必须补一个 `go`：
>
> ```bash
> pyocd cmd --pack <pack> -t STM32G431CBTx -c "init; reset"
> pyocd cmd --pack <pack> -t STM32G431CBTx -c "init; go"
> ```

### 主机单元测试

```bash
gcc -std=c99 -Wall -Wextra -Werror -I Core/Inc \
    Core/Src/foc_hw.c Core/Src/foc_math.c tests/test_foc_math.c -lm -o test_foc
./test_foc
```

---

## 上位机使用

```bash
cd host
python foc_gui.py COM15     # 或双击 run_gui.bat
```

依赖：`pyserial` `matplotlib` `numpy` `tkinter`（Python 3.8+）

功能：串口连接、转速滑条、正/反转/停止、快捷参数面板、
**三路实时曲线**（转速、d/q 电流、母线电压）、原始报文日志。

> ⚠️ **必须用独立 USB-TTL 模块**（CP210x/CH340）接 PB6/PB7。
> **不要用 DAPLink 的虚拟串口** —— 它与 SWD 共用同一个 USB 设备/固件，
> 持续 50 Hz 遥测会让它崩溃，导致 SWD 与串口同时失效。

---

## 串口协议

**MCU → 上位机**（默认 50 Hz，可用 `P period <ms>` 调整）

```
$D,state,fault,spref,speed,id,iq,vd,vq,vbus\r\n
$P,<参数名>,<值>\r\n                       参数设置确认
```

**上位机 → MCU**（行尾 `\n` 或 `\r\n`）

| 指令 | 说明 |
| --- | --- |
| `RUN <hz>` | 设定转速并启动（负值 = 反转）|
| `S <hz>` | 只改参考，不启动 |
| `STOP` | 受控减速停机 |
| `DIS` | 解除使能 |
| `P <名> <值>` | 在线设置参数（26 个）|
| `?` | 立即返回一帧遥测 |

**可在线设置参数**：`rs` `ls` `pllki` `thetaoff` `force` `accel` `starti` `decel`
`octrip` `ocinst` `stallms` `runms` `fw` `fwgain` `fwmax` `obs` `fluxwc` `fluxkp`
`fluxki` `psi` `mtpa` `ld` `lq` `dt` `dtith` `period`

详见 [`host/README.md`](host/README.md)。

---

## 文档

| 文档 | 内容 |
| --- | --- |
| [`docs/FOC_BRINGUP.md`](docs/FOC_BRINGUP.md) | **完整调试记录**：硬件细节、调参历程、每一个踩坑与根因、实测数据、HFI/磁链观测器结论 |
| [`docs/OPEN_LOOP_TEST.md`](docs/OPEN_LOOP_TEST.md) | 开环验证阶段的记录 |

调试过程中定位到的几个**典型坑**（细节见上）：

1. eSMO 中 `Ed * thetaErrSF / Eq_mag` 除零 → NaN 污染角度 → 偶发过流
2. CORDIC 硬件加速的 fallback 分支未读走 RDATA → 偶发返回错误角度 → 失锁
3. 闭环切换点 `SPEED_FORCE` 设在观测器不可靠区（8 Hz）→ 偶发失步
4. 故障停机路径直接短接三相 → 高速时 ~9 A 制动电流冲击
5. 故障后 PI 积分器未复位 → 重启电流失控
6. `pyocd reset` 不启动内核 → 误判为 MCU 死机

---

## 已知问题 / TODO

- [ ] **极对数**：`USER_MOTOR1_NUM_POLE_PAIRS` 继承自参考工程的 `2`，
      但本电机额定约 800 rpm（100 Hz 电频率对应 ~857 rpm）→ 实际极对数应为 **7 左右**，
      需按电机铭牌确认。当前只影响 RPM 显示，不影响控制。
- [ ] **~3 % 空载转速纹波**：需整定死区补偿（`g_foc_dtcomp_td_us`），必须带载进行
- [ ] **磁链观测器**（`g_foc_obs_mode = 1`）：链路已验证，尚未稳定锁定
- [ ] **HFI 低速无感**：高频注入链路正常，尚未闭环
- [ ] 加入 **IWDG** 作为最终兜底（目前靠 CPU 异常处理 + 主循环看门狗）
- [ ] DMA 采样 / 观测器降频（CPU 尚有 ~60 % 余量，暂不必要）

---

## 许可与致谢

本项目代码采用 **MIT License**（见 [`LICENSE`](LICENSE)）。

第三方组件：

- `Drivers/STM32G4xx_HAL_Driver/` — STMicroelectronics，BSD-3-Clause
- `Drivers/CMSIS/` — ARM Limited，Apache-2.0

控制算法参考并移植自 **正点原子 (ALIENTEK) X-STAR-S STM32G4 FOC** 例程，
在此致谢。

---

## English overview

A **sensorless FOC** implementation for a JLC STM32G431 FOC board with an EG2133
gate driver and low-side shunt current sensing.

**Control**: align → I/F open-loop ramp → eSMO sliding-mode observer + PLL →
closed-loop speed control, with field weakening (up to 110 Hz electrical on a 20 V
bus), MTPA, overcurrent/stall protection, a speed-gated fault-stop policy and a
main-loop watchdog.

**Performance**: reliable closed loop from **33 Hz to 110 Hz** electrical,
ISR cost ~3400–4000 cycles (**37–43 % CPU** at 16 kHz / 150 MHz), ~10 s cold start.

**Tooling**: Keil MDK-ARM project, pyOCD flashing, a UART host protocol
(USART1 PB6/PB7, 115200) and a Python/tkinter GUI under `host/`.

**Docs**: the full bring-up log — including every bug found and its root cause —
is in [`docs/FOC_BRINGUP.md`](docs/FOC_BRINGUP.md).

**License**: MIT.
