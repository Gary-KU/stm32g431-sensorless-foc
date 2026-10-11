# 无感 FOC（eSMO）移植与调试记录

本文档记录把 **ALIENTEK X-STAR-S G431 FOC 的 eSMO 无感 FOC** 移植到
本板（嘉立创 STM32G431 无刷 FOC 开发板）的过程、当前状态、调参入口和调试方法。

---

## 1. 总体架构

算法来自参考工程 `X-STAR-S_STM32G4_FOC_SMO/FOC - 3 - FSMO`（TI InstaSPIN 风格）。

| 文件 | 内容 |
| --- | --- |
| `Core/Src/user_vars.c` `Core/Inc/user_vars.h` | 参数结构、PI、ESMO/PLL 参数、SPDFR 参数 |
| `Core/Src/user_motor.c` `Core/Inc/user_motor.h` | CLARKE/PARK/IPARK/SVGEN/PI/TRAJ/ANGLE_GEN/SPDFR/ESMO，数学常量 |
| `Core/Inc/user_mtr.h` | 电机参数、PWM/时钟、增益、缩放系数 |
| `Core/Src/motor_foc.c` `Core/Inc/motor_foc.h` | **本板硬件层 + 启动状态机 + 黑匣子** |
| `Core/Src/foc_hw.c` `Core/Inc/foc_hw.h` | CORDIC/三角函数封装（当前用 libm，见第 6 节） |
| `Core/Src/foc_math.c` `Core/Inc/foc_math.h` | Clarke/Park/SVPWM 的自测版本（主机测试用） |

`user_foc.c`（参考的硬件+应用层）**没有**移植，由 `motor_foc.c` 取代。

### 控制链路

```
ADC(U,V) → 重构 W=-(U+V) → Clarke → Park(θ) → PI(Id,Iq) → InvPark(θ) → SVGEN → TIM1
                                    ↑ θ
        eSMO: 滑模电流观测器 → 反电动势 LPF → PLL 锁相 → θ, ω
```

### 启动状态机（`MOTOR_Status_e`）

| 状态 | 行为 | 退出条件 |
| --- | --- | --- |
| `ALIGNMENT` (6) | 固定 θ=0，`Id=alignCurrent_A`(0.5A)，`Iq=0` | 2s |
| `OL_START` (8) | 开环角 `angleGen`，`Iq=startCurrent_A`(0.5A)，按 `ACCEL_START_Hzps` 爬速 | 估计转速 ≥ `SPEED_FORCE_Hz` 且延时到 |
| `CL_RUN` (9) | θ 切换到 PLL 角度，电流环闭环 | `startupTimeDelay`(2s) |
| `CTRL_RUN` (10) | 速度环使能 | — |
| `NORM_STOP` (3) | 自动停机（`g_foc_run_ms`）或 PB10 | — |

---

## 2. 硬件层（本板专有）

| 项 | 配置 |
| --- | --- |
| 主频 | **150 MHz**（HSI 16M /4 ×75 /2）；**不用 170MHz boost**，见第 6 节 |
| PWM | TIM1 中心对齐 **16 kHz**，ARR=4687，RCR=1 |
| 输出 | CH1=PA8=W，CH2=PA9=V，CH3=PA10=U，**单端**（EG2133 内部互补+死区） |
| 电流采样 | **ADC1 规则转换**，TIM1 更新中断里软件触发：U=CH5/PB14，V=CH14/PB11 |
| 第三相 | W 相采样硬件坏 → `Ic = -(Ia+Ib)`（2 采样 FOC） |
| 母线 | ADC2 CH15/PB15，1 ms 轮询 |
| 电流缩放 | `USER_M1_ADC_FULL_SCALE_CURRENT_A = 20.0`（10 mΩ × 16.5） |
| 电压缩放 | `USER_M1_ADC_FULL_SCALE_VOLTAGE_V = 36.3`（100k/10k，×11） |

> ⚠️ **W 相电流采样硬件故障**：运放 U9.2 → R41 → PB12 只有约 0.093V（正常应 ~1.62V）。
> 已用 `-(Ia+Ib)` 重构绕过。修好后可改回 3 采样。

---

## 3. 关键参数（`Core/Inc/user_mtr.h`）

```c
#define USER_MOTOR1_NUM_POLE_PAIRS      (2)
#define USER_MOTOR1_Rs_Ohm              (1.25f)          // 实测线间 2.5Ω ÷2
#define USER_MOTOR1_Ls_d_H              (0.00156f)       // HF 扫描实测
#define USER_MOTOR1_Ls_q_H              (0.00239f)       // 凸极率 Lq/Ld = 1.53
#define PWM_FREQUENCY                   (16000)
#define USER_M1_MAX_VS_MAG_PU           (0.55f)
#define USER_MOTOR1_ALIGN_CURRENT_A     (0.5f)
#define USER_MOTOR1_STARTUP_CURRENT_A   (0.5f)
#define USER_MOTOR1_TORQUE_CURRENT_A    (0.4f)
#define USER_MOTOR1_MAX_CURRENT_A       (1.0f)
#define USER_MOTOR1_FREQ_MAX_Hz         (200.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz      (8.0f)      // 强拖→闭环切换点
#define USER_MOTOR1_ACCEL_START_Hzps    (1.5f)
#define USER_MOTOR1_ACCEL_MAX_Hzps      (4.0f)
#define USER_MOTOR1_KSLIDE_MAX/MIN      (0.50f/0.50f)  // 恒定最大增益
#define USER_MOTOR1_PLL_KP_MAX/MIN      (10.0f/1.50f)
```

### 运行时可调（不用重新编译！）

`g_foc_rs_ohm`、`g_foc_ls_h` 是 `volatile float` 全局变量，
**每次启动时**由 `MOTOR_vars_Init()` 重新读取。
调试器里改完值，重新 arm 一次即可生效（不用复位、不用重编）。

---

## 4. 调试基础设施

| 机制 | 说明 |
| --- | --- |
| **黑匣子** | `g_foc_log[256][4]` 每 100 ms 记 `{state, speed*100, Iq*1000, Vs*10}`，共 25.6 s |
| **自动停机** | `g_foc_run_ms`（ms，0=永久）。跑完自动刹停 → PWM 停 → SWD 恢复 → 读黑匣子 |
| **PB10 急停** | 低电平立即刹停（不依赖调试器） |
| **DBGMCU 冻结** | 调试器 halt 时 TIM1 停 → PWM 停 → 可安全单步 |
| **SWD 限制** | ⚠️ **PWM 运行时 SWD 会被开关噪声打断**。所以铁律：*arm 后立刻断开，跑完再读* |

### 4.1 调试读取“坏数据”的两个真实原因（重要教训）

调后期出现过多次“读到错误值”的困扰，实际原因只有两个：

**原因 A：重编译后符号地址会移位**
新增/删除一个变量或源文件，`.data`/`.bss` 的布局就会变，**所有全局变量的地址都会变**。
沿用旧地址去读，就会读到别的变量（往往是存的函数指针/常量，看着像 `0x0800xxxx` flash 地址，
极容易误判为“MCU 坏了”）。

→ **每次改代码重新编译后，必须重取地址**：
```bash
arm-none-eabi-gdb -batch -ex "file Mymotor.axf" -ex "p/x &g_foc_log" -ex "p/x &g_foc_log_idx" ...
```
或直接查 `Mymotor.map`。

**原因 B：长连接的 pyocd 会话会变陈旧**
在 Python 里用一个 `session` 长时间反复读写，后期会返回**恒定错误值**（例如所有地址都返回同一个值）。

→ **优先用独立进程 `pyocd cmd`** 读取，或**每次重开 session**；关键值读两遍校验。

**不是 DAPLink 坏了，也不是 MCU 坏了。** 判断方法：换独立进程重读，值就正常了。

### 4.2 远程操作流程

```python
# 1) arm（halt → 写变量 → resume → 立刻断开）
t.halt()
t.write32(0x2000004c, 0x41A00000)  # g_foc_speed_ref_hz = 20.0f
t.write32(0x20000080, 20000)       # g_foc_run_ms = 20 s
t.write32(0x20000048, 1)           # g_foc_command: 1=正转 -1=反转 2=停
t.write32(0x20000044, 1)           # g_foc_arm = 1
t.resume(); session.close()        # 立刻断开！
# 2) 等 run_ms + 5 s，重连读 g_foc_log / g_foc_state
```

---

## 5. 实测结果

| 项目 | 结果 |
| --- | --- |
| 电流环（ALIGN，Id 跟随 0.5A） | ✅ 稳定 |
| I/F 强拖启动（Iq=0.5A，转速平滑爬升） | ✅ |
| 观测器锁相 + 切闭环 | ✅ |
| 速度环稳定锁定 **60 Hz（精确跟随指令）** | ✅ 波动 ±0.1 Hz |
| 高速段（80–100 Hz） | ✅ |
| 20–60 Hz | ✅ |

**实测**：ALIGN(Id=0.5A) → I/F 强拖(转速平滑爬升 0→7Hz) → 切闭环 → 平滑升速 → **精确锁定 60Hz，Vs=6.6V 从容**。

**16V 母线上限**：指令 100Hz 时平滑爬升到 **~85 Hz** 后受电压限制自然顶住（BEMF ≈ 9.3V 对上 VsMax 8.8V），
电流仍只有 ~0.1A。想上 100Hz+ 需要**提高母线到 20–24V**（板子标称支持）。

**反转**：-40Hz 指令 → 实际到 **-39.6Hz**，停机平滑。

### 5.3 验证记录

| 项目 | 结果 |
| --- | --- |
| 60 Hz 稳态 | 锁定 60.0 Hz，波动 ±0.1 Hz；Vs=6.6 V，Iq≈0.1 A |
| 高速（16 V） | ~85 Hz（16 V 电压上限）| 
| **高速（20 V + 弱磁）** | **109.8 Hz ≈ 3300 rpm**（无弱磁约 100 Hz）✅ |
| 反转 | -39.6 Hz ✓ |
| 受控减速停机（20/40/反转/110Hz）| 平滑降速（~30 Hz/s），`fault=0`，无冲击 ✓ |
| 过流保护 | 阈值临时改 0.3 A → 立即 `FAULT=1` ✓ |
| 堵转保护 | 已实现，未实机验证 |
| HFI | 注入/解调链路已验证，闭环跟踪未锁定 ❌ |

### 5.4 弱磁（Field Weakening）

按 Morimoto 1990 的经典方案：当 `|V|` 接近 `0.95·VsMax` 时，用 PI 累积出负的 `id` 抵消磁链。

- 运行时变量：`g_foc_fw_enable`(1)、`g_foc_fw_gain`(0.0002 A/(V·ISR))、`g_foc_fw_max_a`(0.8 A)
- 同时把总电流限制在 `sqrt(id²+iq²) ≤ maxCurrent_A`
- **停机（`stop_req`）时自动退出弱磁**，避免制动时电流叠加误触发过流

**实测效果（20 V）**：
```
转速   ~100 Hz (无FW)  →  109.8 Hz (FW，id = −0.62 A)
```
与理论一致：`Ld·|id| = 1.56mH×0.62A = 0.001 Wb`，占磁链(0.0175 Wb)约 6%。

**想更高转速**：把 `g_foc_fw_max_a` 调大（受 `maxCurrent_A` 约束）或提高母线。

---

## 5.1 停机行为（重要安全点）

**本板无法「滑行停车」**：EG2133 的 HIN 与 LIN# 接在一起，每相只能「上管开」或「下管开」，
**两管全关不可能**。所以一旦关闭 PWM 就只能靠**下管短接制动**。

短接制动时电流**不受电流环/电源限流控制**，由反电动势和绕组电阻决定：
`I = Ke·ω / R`，40 Hz 时约 3.5 A，60 Hz 时约 5.3 A → **高速下会「哐」一下 + 大电流冲击**。

**解决：受控减速**（`g_foc_stop_decel_hzps`，默认 30 Hz/s）
- 停机时**保持 FOC 运行**，把速度指令斜坡降到 0 → 速度环反向制动，**电流环把制动电流限制在 ≤maxCurrent_A**
- 等转速 < 3 Hz（或超时 5 s）**才**下管短接停住
- PB10 急停键、`g_foc_run_ms` 自动停机、`g_foc_command=2` **都走这条受控路径**
- 只有**故障**（过流/堵转）才立即短接制动

## 5.2 保护

| 保护 | 触发条件 | 动作 |
| --- | --- | --- |
| 过流 | 相电流峰值 > `g_foc_oc_trip_a`(1.5A) 持续 1 ms，或 > `g_foc_oc_inst_a`(3A) 瞬时 | 立即制动，`g_foc_fault=1` |
| 堵转/失步 | 速度环输出 `IsRef_A` 顶在 0.9×maxCurrent_A 超过 `g_foc_stall_ms`(400ms) | 立即制动，`g_foc_fault=2` |

`g_foc_fault` 非 0 时 LED(PC13) 闪烁，`g_foc_command=2` 清除。

## 6. 已知问题 / 待办

### 6.1 CORDIC PHASE 函数不一致（已绕过）
参考工程用 CORDIC 的 `PHASE` 函数算 LPF 相位补偿 `thetaOffset`。
本板 CORDIC 返回值与参考不一致。**最终解决**：不修 CORDIC，而是
**彻底不用 `thetaOffset`**（把解调角改成 PLL 自己的估计 `thetaEst`），见 6.3。

### 6.2 170 MHz 不稳
170 MHz（Range-1 boost）下 SWD 极不稳定，怀疑该板电源/去耦不足。
已降到 **150 MHz**（不用 boost），SWD 明显稳定。

### 6.3 观测器丢锁（已解决 ⭐）
**根因**：参考工程的 PLL **积分项被关死**（`pll_Ki = 0`），靠 CORDIC 算的
`thetaOffset` 补稳态相差；且积分项**没有限幅** → 强拖阶段 BEMF 太小 → 积分饱和
→ 转速估计顶死 → 过早切闭环且转速失控。

**修复（3 处）**：
1. `ESMO_run`：解调角用 PLL 自己的估计 `obj->thetaPll = obj->thetaEst`（不用 `theta+thetaOffset`）
2. `ESMO_init`：**打开积分项** `pll_Ki = g_foc_pll_ki`（默认 0.0005）→ PLL 变 II 型，稳态相差为零
3. **抗饱和**：`pll_ui = MATH_sat(pll_ui + Ki*err, Umax, Umin)`；且强拖阶段 `pll_ui = 0`

效果：上限从 **24 Hz → 100 Hz**，60 Hz 下波动 ±0.1 Hz。

### 6.4 速度斜坡
`MOTOR_vars_Init` 里 `trajHandle_spd.maxDelta` 用的是 `USER_M1_MAX_ACCEL_Hzps`(2.0)，
而运行中 1 ms 循环会按 `USER_MOTOR1_ACCEL_MAX_Hzps`(4.0) 更新。已修。

### 6.5 其它
- **过流保护**：每 ISR 检查三相电流幅值，**1.5A 持续 1ms** 或 **3A 瞬时** 立即刹停并锁存
  `g_foc_fault=1`（LED 闪烁），阈值可用 `g_foc_oc_trip_a` / `g_foc_oc_inst_a` 在线改。已验证能动作
- 无堵转/失步保护（失步时电流不一定超限）
- 正反转切换未测
- `user_foc.c` / `foc_math.c` / 开环相关文件仍在工程里但未参与 FOC 运行

---

## 6.6 运算开销与硬件加速

实测（DWT 周期计数器 `g_foc_isr_cycles` / `_max`，16 kHz ISR，预算 9375 cycle @150 MHz）：

| 阶段 | 典型 | 峰值 | CPU 占用 |
| --- | --- | --- | --- |
| 初版（libm 三角 + 死代码）| 3852 | 4652 | 41% / 50% |
| **CORDIC sincos + 删死代码后** | **3683** | **3960** | **39% / 42%** |

编译器已为 **`-O3`**（Keil `<Optim>4</Optim>`）。

**已启用的硬件加速**：
- FPU：全部浮点运算
- **CORDIC**：`FOC_SinCos()` 用 “Sine + 2 结果” 一次算出 cos 与 sin（foc_hw.c）
  - ❗ **读数顺序：第一次 RDATA 是 sin，第二次是 cos**（已由 `FOC_Hw_SelfTest` 验证）
  - 精度与缩放已验证：sin30=0.5, cos30=0.866, sin90=1, cos90=0

**尚未启用**（CPU 有余量，非必须）：
- ADC 用 DMA + 硬件触发（现在是阻塞轮询，~200-300 cycle）
- 观测器降频（如 8 kHz）
- FMAC 做 BEMF/速度滤波

**测量方法**：DWT CYCCNT 在 ISR 首尾取值，存 `g_foc_isr_cycles`/`g_foc_isr_cycles_max`，用 `pyocd cmd` 读。

### 6.7 潜伏的除零 Bug（重要）

`user_motor.c` 的 eSMO 里（移植自参考工程）：
```c
obj->thetaErr = obj->Ed * thetaErrSF / obj->Eq_mag;   // 旧代码，无保护
```
`Eq_mag = √(Eα²+Eβ²)` 在反电动势矢量过零时（I/F→闭环切换、各种瞬态）为 0，
除法产生 **NaN**，污染 PLL 角度 → 电流环失控 → **过流保护跳闸**。

这是之前多次“偶发过流/危险”的真正元凶（包括手动掰轴、切闭环、加 MTPA 后复现）。

**修复**：
```c
if (obj->Eq_mag > 1.0e-6f) obj->thetaErr = obj->Ed * thetaErrSF / obj->Eq_mag;
else                        obj->thetaErr = 0.0f;
```
修复后同样的 40 Hz 运行 `fault=0`，稳定。

### 6.8 MTPA / FMAC / DMA 评估

| 项 | 结论 |
| --- | --- |
| **MTPA** | 已实现（`g_foc_mtpa_enable`，含 NaN 与范围保护）。但本电机 ψr 大、ΔL 小，id 仅 ~0.1 mA@空载 → **收益可忽略**，保留无害 |
| **DMA** | 真实收益仅 ~200 cycle（ISR 的 5%）。约需重配 DMAMUX + ADC 扫描链，**收益小，暂不做** |
| **FMAC** | **负收益**：本工程滤波器均为一阶（~5 cycle），FMAC 每次调用开销 20-30 cycle。**不做** |

**实时 ISR 开销**（测量法见 6.6）：典型 ≈ 3975 cycle，峰值 ≈ 4167 → CPU 占用 ~43%。

### 6.9 CORDIC 引起偶发失锁（已回退 libm）

启用 CORDIC 加速 sin/cos 后，出现**偶发失锁**（`fault=2`，速度估计反号，
40 Hz 跑 5-8 s 后跳闸）。根因是实现的 fallback 分支：

```c
while (((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U) && (guard != 0U)) guard--;
if (guard == 0U) { /* 回退 libm */ return; }   // ← 没读走 RDATA！
```

轮询超时时**未读走结果寄存器**就返回，下一次写 WDATA 与上一笔未读结果
竞争 → 偶发返回**错误角度** → Park/InvPark 出错 → 转矩冲击 → 观测器失锁。

**处置**：ISR 热路径**回退到 libm**（`FOC_SinCos`），CORDIC 保留配置并仅在
`FOC_Hw_SelfTest()` 中验证硬件本身。代价：CPU 从 39% → **37%**（可忽略）。
回退后 40 Hz 长测 30 s 全程 `fault=0`。

**教训**：硬件加速器一旦进入安全相关的热路径，必须有"异常时也不破坏状态"
的保证；省 4% 的 CPU 不值得冒丢锁的风险。

### 6.10 死区补偿（已实现，需带载才能整定）

已在 αβ 域实现补偿（`g_foc_dtcomp_td_us`，0=关闭）：
```c
Vα += dv*(2/3)*(sa - 0.5*sb - 0.5*sc);   Vβ += dv*(1/√3)*(sb - sc);
dv = (Td/Ts)*Vbus,  sa/sb/sc = sign(ia/ib/ic) with a threshold
```

**空载整定失败**：空载相电流峰值仅 ~0.07 A，而符号阈值 `g_foc_dtcomp_ith_a`
默认 0.05 A → 大部分时间落在不补偿的"死区"，只在波峰附近补偿 →
**Iq 纹波从 0.047 A 恶化到 0.17 A**。

**结论**：死区补偿必须**带载**（电流远大于阈值）才能整定，且主要在低速有意义。
默认保持关闭（`td=0`）；待有负载 / 低速观测器可用后再整定。

### 6.11 低速极限与闭环切换点（根因）

用运行时覆盖量 `g_foc_speed_force_hz`（不会被启动重置）下探闭环下限，
每个目标速度同时作为切换点：

| 目标 | 结果 | Iq_pp |
| --- | --- | --- |
| 30 Hz | ✅ 稳定 | 0.043 A |
| **25 Hz** | ✅ **稳定（切换点选用此值）** | 0.042 A |
| 20 Hz | ⚠️ 勉强，速度在 18~22 抖 | 0.18 A |
| 15 Hz | ❌ 拉不上去，卡在 10.85 Hz | 0.28 A |
| 12 Hz | ❌ 失步 `fault=2`，速度估计跳至 **−44.5 Hz** | 1.08 A |
| 10 Hz | ❌ 起不来 | — |

**根因**：编译期 `SPEED_FORCE = 8 Hz` 把闭环接管点放在 eSMO 的不可靠区
（8 Hz 处反电动势仅 ≈1 V）→ 观测器锁不住 → 偶发失步/震荡/过流。
**这是之前所有"偶发故障"的根本原因**（不是 CORDIC，CORDIC 是另一个独立问题）。

**修复**：`USER_MOTOR1_SPEED_FORCE_Hz` 8.0 → **25.0** Hz。
验证：40 Hz 运行 50 s 全程 `fault=0`，稳态 40.0±0.1 Hz，Iq 纹波 ~0.05 A。

**含义**：本套 eSMO 无感方案的**可靠闭环下限 ≈ 25 Hz**（机械 ≈ 750 rpm，pp=2）。
再低只能靠 I/F 开环强拖；要真正下降需更好的低速观测器（磁链/HFI）。

## 7. 上位机 / 串口链路

### 7.1 引脚与硬件

USART1 在 **PB6 (TX) / PB7 (RX)**，115200 8N1。PA9/PA10（另一组 USART1 引脚）
已被 TIM1 PWM 占用，所以 PB6/PB7 是本板唯一可用的映射。

**必须用独立 USB-TTL 模块（CP210x/CH340）**，不要用 DAPLink 的虚拟串口：
DAPLink 的 VCP 与 SWD 共用同一个 USB 设备与固件，持续 50 Hz 遥测会让它
崩溃，**SWD 与串口同时失效**（实测复现一次，拔插 USB 后恢复，MCU 无恙——
FOC 的 `run_ms` 自主保护已自行停机）。

### 7.2 协议

```
MCU → PC (50 Hz, 可用 P period 调):  $D,state,fault,spref,speed,id,iq,vd,vq,vbus
PC → MCU (行尾 \n):
  RUN <hz> / R <hz>   设定转速并启动（负值=反转）
  S <hz>              只改参考
  STOP / DIS          停机 / 解除使能
  P <名> <值>         设置运行参数（22 个）
  ?                   立即回一帧
```

### 7.3 实现

| 文件 | 说明 |
| --- | --- |
| `Core/Src/foc_serial.c` | USART1 + 接收中断环形缓冲 + 行解析 + 遥测发送 |
| `Core/Inc/foc_serial.h` | 接口与协议说明 |
| `host/foc_gui.py` | tkinter + matplotlib 上位机 |
| `host/run_gui.bat` | 双击启动 |
| `host/README.md` | 协议与使用说明 |

细节：
- USART1 中断优先级 **3**，低于 16 kHz 的 FOC 中断（TIM1 = 1），不会挤占控制环。
- 发送自实现整数化浮点格式化（`fmt_f`），**不依赖 libc 的 `%f`**
  （嵌入式上 microlib 不支持，标准库又会拉进大块代码）。
- 参数回显按数值量级自适应小数位，避免 `0.0005` 被四舍五入成 `0.001`。

### 7.4 实测启动时序（20 V, 目标 40 Hz，全部经串口完成）

```
t= 3s  OL_START    7.37 Hz  Iq=0.498
t= 6s  OL_START   18.72 Hz  Iq=0.536
t= 9s  CL_RUN     40.52 Hz  ← 25 Hz 切换成功
t=12s  CTRL_RUN   42.26 Hz  ← 速度环接管
t=20s  CTRL_RUN   40.07 Hz
t=25s  CTRL_RUN   39.97 Hz  ← 稳态 ±0.1 Hz
t=30s  CTRL_RUN   18.30 Hz  ← run_ms 到，受控减速
t=35s  NSTOP       3.00 Hz
停机后 IDLE  fault=0
```

启动约 **9 秒到 40 Hz**（原来是 17 s+）。为此同时修了两处：

1. **`ACCEL_START` 之前完全无效** —— `trajHandle_spd.maxDelta` 在 OL_START 分支
   保持初始化值，`accelerationStart_Hzps` 从未被使用。现在 OL_START 分支显式
   用 `accelerationStart_Hzps` 设定，并把默认值 1.5 → **4.0 Hz/s**。
2. **`g_foc_run_ms` 默认 12000 太短** —— I/F 要到 25 Hz 需 ~17 s，旧默认值会让
   电机**在切换闭环之前就自动停机**。默认改为 **30000 ms**。

## 8. HFI（低速无感）— 阶段性结论

**已确认可行的部分**：
- 电机有真实凸极：静态扫描（转子用直流电流夹住）得 **Ld=1.56 mH, Lq=2.39 mH，凸极率 1.53**，
  L(θ) 呈平滑 180° 周期曲线
- 高频注入链路正常：1 kHz 正弦注入，HF 电流 ~0.08 A（符合理论）
- 解调链路存在响应（冻结角度扫描时 err 随 Δθ 单调过零）

**没能做成的部分**：实时解调出的位置信号太弱，**闭环跟踪无法锁定**。

**根因分析**：
1. 凸极项只占 HF 电流的约 1/3 → **~0.024 A ≈ 5 个 ADC 码**，信噪比很低
2. **16 kHz 的 PWM 纹波与测量混叠**，淹没了 1 kHz 的 HF 响应
3. 静止时脉振注入会“磁锁”转子（正常现象），使 Δθ 恒≈0

**若要继续，关键突破口**：
- **改进采样时序**：与 PWM 同步、去除纹波混叠（现在是每个 PWM 周期软触发一次）
- 提高注入幅度/选更优注入频率与解调方式（带通 + 双解调）
- 需要多轮实机调试

**为什么可以先放下**：I/F 强拖已经能 0 速起步，HFI 的边际收益只是“更低速时的角度精度”。

### 8.1 观测器升级（磁链观测器）— 阶段性结论

按经典思路加了一个**定子磁链观测器**（与 eSMO 并存，`g_foc_obs_mode` 切换）：
```
ψ = ∫(v − Rs·i)dt   （带泄漏的低通积分，防漂移）
转子磁链 = ψ − Ls·i
θ = atan2(ψβ − Ls·iβ , ψα − Ls·iα)  → PLL
```
理论上它的信号在低速**不趋于 0**（不像反电动势），应该能把无感下限压到 I/F 切换点以下。

**结果：未能锁定 ❌** —— 在 I/F → 闭环切换那一刻（~8 Hz）电机就停（`fault=2 堵转保护`）。

**已知失败点**：
1. 磁链积分的**初值**：电机实际磁链是 `L·i + ψr`，初值不对 → 角度错
2. 相位/符号约定需校：`atan2(ψrβ, ψrα)` 是否就是 FOC 需要的 d 轴角
3. PLL 增益（`g_foc_flux_kp/ki`）与泄漏极点 `g_foc_flux_wc` 未整定
4. 低速时 `v − R·i` 两项都很小，积分对 `Rs` 误差与 ADC 偏置很敏感

**结论**：**eSMO 版本（`g_foc_obs_mode = 0`，默认）仍然是当前可用方案**，
磁链观测器作为后续可选方向保留（要投入几轮整定）。

## 9. 下一步计划

### 9.1 高速/启动稳定性排查（本轮）

#### 🔴 工具链陷阱：`pyocd reset` 不启动 MCU

多次出现“烧写/复位后串口无数据、SWD 也连不上”的现象，一度误判为 MCU 死机。
实际原因：**`pyocd reset` 之后内核处于未运行状态**，代码根本没跑。
用 `pyocd cmd -c "init; go"` 后遥测立即恢复。

```bash
pyocd flash ... && pyocd cmd ... -c "init; reset" && pyocd cmd ... -c "init; go"
```

**以后烧写后必须补一个 `go`，并验证遥测。**

#### 🔴 SPEED_FORCE 提到 33 Hz

25 Hz 仍处于 eSMO 可靠边缘，切闭环**时好时坏**（有时在 10 s 处失步）。
提到 **33 Hz** 后，35/60/100 Hz 启动 **100% 成功，零失步**。

#### 🔴 故障路径直接短接（危险设计缺陷）

正常停机是受控减速，但**故障路径（stall/fault=2）直接 
`MotorFOC_Brake()` 下管短接**。100 Hz 时反电动势 ~11 V，短接制动电流
≈ 11/1.25 ≈ **9 A** —— 正是本文档早就警告过的冲击。

新增 `FaultStop()` 统一策略：

| 情况 | 动作 |
| --- | --- |
| 堵转/失步 且 \|speed\| > 15 Hz | **受控减速**（电流环限流）|
| 堵转/失步 且 \|speed\| ≤ 15 Hz | 下管短接 |
| 瞬时过流 > 4 A | 立即切断 |
| 持续过流 | 受控减速 |

#### 🟡 故障后 PI 积分器不复位

故障后重新启动会继承卷绕的 Id/Iq/速度 PI → 电流环失控（实测 Id 摆 ±0.6 A、
Iq 达 1.4 A，而指令仅 0.5 A）。已在重新使能时清零三个 `.Ui`。

#### 🟡 弱磁过激 → 转速失控

`fw_max_a = 0.8 A` 削磁过猛 → 100 Hz 参考下转速冲到 **108 Hz** → 反电动势
塔陷 → 观测器失步。改为 **0.35 A**，并加入“实际转速高于参考时回退弱磁”
的项。100 Hz 的主导振荡从 3.6 Hz/幅值 1.48 降到 9.2 Hz/幅值 0.90。

#### 🟡 速度环喂的是未滤波的 PLL 速度

`speed_reg_Hz = speed_Hz`（原始值）→ 观测器噪声直接进 PI。改为用
`speedFilter_Hz`（滤波级也放缓到 ~1 ms）。改善有限（3.2%→2.8%），
说明**残下的 ~3% 转速纹波来自电流/转矩纹波本身**，而非控制环噪声。

#### 实测结果（20 V 母线，空载）

| 目标 | 结果 | std | 峰峰 | 主导波动 |
| --- | --- | --- | --- | --- |
| 35 Hz | ✅ 稳定 | 1.16 (3.2%) | 4.5 Hz | 38 Hz (0.04，ωe 纹波已极小) |
| 60 Hz | ✅ 稳定 | 1.71 (2.9%) | 6.5 Hz | — |
| **100 Hz** | ✅ **稳定（不再失步）** | 2.72 (2.8%) | 10.4 Hz | 9.2 Hz (0.90，弱磁交互) |

**残下问题**：~3% 转速纹波。空载时 Iq 均值仅 ~0.05 A 而纹波 ~0.02 A（相对 40%），
小惯量下转矩纹波直接反映为转速纹波。**带载后相对纹波会显著下降**；
要空载改善需整定死区补偿（需带载才能做，参见 6.10）。

### 9.3 ★ 高速不稳的真正根源：弱磁回路（已默认关闭）

上一节的~3% 转速纹波一直归因于“电流/转矩纹波”，实际**不是**。
在 100 Hz 扫描弱磁增益后真相很清楚：

| fwgain（fwmax=0.35 A）| 转速 std | 主导波动 |
| --- | --- | --- |
| 0.0002（原默认）| 4.11 % | 4.3 Hz（幅值 1.39）|
| 8e-05 | 6.38 % | 3.8 Hz（6.75）|
| 3e-05 | 6.67 % | 4.0 Hz（7.28）|
| **0（关闭）** | **1.41 %** | 7.2 Hz（0.09）|

**弱磁积分器与速度环构成一个阻尼不足的回路，向整个转速范围注入 ~4 Hz 振荡。**
而它的收益只有几个百分点：20 V 母线的电压极限基础转速已经是 ~105 Hz，
弱磁只能多换 ~5%（且 `fwmax=0.35 A` 只能削磁 `Ld·id=3 %`）。

**处置：`g_foc_fw_enable` 默认改为 `0`**。需要更高转速时可 `P fw 1` 打开，
但要接受纹波回来。

#### 最终实测（弱磁关 + 原始速度反馈）

| 目标 | 结果 | std | 峰峰 | 主导波动 |
| --- | --- | --- | --- | --- |
| 35 Hz | ✅ | **0.42 %** | 0.71 Hz | 38 Hz (0.08) |
| 60 Hz | ✅ | **0.22 %** | 0.64 Hz | 65 Hz (0.05) |
| 80 Hz | ✅ | **0.84 %** | 3.61 Hz | 87 Hz (0.10) |
| 100 Hz | ✅ | **0.77 %** | 4.09 Hz | 6.8 Hz (0.05) |
| 110 Hz | ❌ 达不到（需开弱磁）| — | — | — |

**相比弱磁开启时的 3–4.6 %，改善 4–15 倍。**

#### 速度环反馈源：原始值比滤波值好

供参考，同一试验下对比：

| 反馈 | 100 Hz std | 100 Hz 峰峰 | 阶跃超调 | 阶跃后 std |
| --- | --- | --- | --- | --- |
| 滤波值（~1 ms）| 4.60 % | 19.88 | +0.71 | 0.293 |
| **原始 PLL 速度** | **4.29 %** | **12.95** | **+0.52** | **0.145** |

滤波给速度环带来的相位滞后反而恶化了动态。已改为**默认用原始值**
（运行时可 `P spdfb 0/1` 切换；滤波值仍用于显示与遥测）。

#### 一个重要教训

排查时我一度用“阶跃响应”测稳定性，但因为加速度限幅是 `4 Hz/s`，
30 Hz 的阶跃需要 7.5 s，而我只采集了 3 s —— **量到的是斜坡中段，不是超调**，
因此得出“弱磁上限砍掉了高速能力”的错误结论。**测动态响应必须覆盖完整斜坡。**

### 9.5 启动可靠性（“老是起步有问题”）

症状：连续启动测试 **0/10 成功**，多数 `FAULT=2`（堵转/失步）。转子在 I/F 阶段
只爬到 2~18 Hz，而轨迹已到 33 Hz 切换点。

已做的修改：

| 项 | 原值 | 新值 |
| --- | --- | --- |
| I/F 强拖电流 | 0.5 A | **0.9 A** |
| I/F 斜坡 | 4.0 Hz/s | **3.0 Hz/s** |
| 定位电流 | 0.5 A | **0.8 A** |
| 切换闭环 | 只看转速 | 额外要求 \|thetaErr\| < 0.30 |

**关键发现：启动需要足够的静止时间**

| 起停间隔 | 结果 |
| --- | --- |
| 6 s | ❌ 0/5、1/3 |
| **15~30 s** | ✅ **3/3 + 4/4 全部成功** |

推测是热/稳定效应（电流采样零点漂移、磁体温度）。**实用结论：起停间留
15~30 秒。** 根本解法是 HFI（见 §8）。

### 9.6 ★ 堵转保护在变速时误触发（“运行一会儿就慢慢停下来”）

症状：运行中（尤其**改变转速**后）电机**平滑地慢慢停住**。

**根因**：堵转判据是“电流指令持续顶在限幅上 g_foc_stall_ms 以上”，
但**加速时速度环本来就必须顶到限幅**（要加扭矩才能加速）。斜坡稍快或惯量
稍大 → 400 ms 后误判为堵转 → `FaultStop()` 在高速用受控减速 → “慢慢停”。

**修复**：只在**轨迹已走到目标**时才判定：

```c
float traj_err = fabsf(trajHandle_spd.intValue - trajHandle_spd.targetValue);
if (traj_err > 1.0f)  stall_cnt = 0U;     /* 还在斜坡，不算堵转 */
else if (...IsRef_A > 0.9*maxCurrent...)   /* 原判据 */
```

**同时加快斜坡**（用户反馈）：

| 参数 | 原值 | 新值 |
| --- | --- | --- |
| 运行中加/减速 | 4.0 Hz/s | **8.0 Hz/s**（可 `P accmax` 调）|
| I/F 启动斜坡 | 2.0 Hz/s | **3.0 Hz/s**（启动 21.5 → **15.0 s**）|
| 停机减速 | 30 Hz/s | 不变 |

**验证**：启动 40 Hz 后连续变速 80 → 50 → 90 → 40 → 70 Hz，5/5 全部无保护动作，
转速精确跟随（80.52 / 50.25 / 89.81 / 40.17 / 69.32）。

### 9.4 待办

1. **拿到电机 Ls**（关键）→ 已用 HF 辨识实测：**Ld=1.81mH, Lq=2.30mH, 凸极率 1.27**
2. 用 `g_foc_rs_ohm`/`g_foc_ls_h`/`g_foc_pll_ki` 在线微调
3. (done) eSMO/PLL 已修好，可锁 100Hz
4. **实现 HFI**（凸极率 1.27 可行）→ 零速/低速无感
5. 加过流/堵转保护
6. 正反转 + 运行中实时变速测试
