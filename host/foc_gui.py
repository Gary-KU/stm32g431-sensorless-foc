#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mymotor FOC upper computer (上位机)
====================================
Talks to the STM32G431 FOC board over USART1 (PB6=TX / PB7=RX, 115200 8N1).

The link is normally the DAPLink virtual COM port (VID 0D28:0204).

Protocol
--------
  MCU -> host  ~50 Hz :  $D,state,fault,spref,speed,id,iq,vd,vq,vbus
  host -> MCU         :  RUN <hz> | STOP | DIS | S <hz> | P <name> <val> | ?

Run:  python foc_gui.py            (or python foc_gui.py COM35)
"""

import queue
import sys
import threading
import time
from collections import deque

import serial
import serial.tools.list_ports as list_ports

import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use("TkAgg")
# Chinese font: the default DejaVu Sans has no CJK glyphs, which makes every
# Chinese label render as a tofu box.  Pick the first available CJK-capable
# family shipped with Windows.
matplotlib.rcParams["font.sans-serif"] = [
    "Microsoft YaHei", "SimHei", "SimSun", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import tkinter.font as tkfont


def apply_cjk_font(root):
    """Give every tk default font a CJK-capable family."""
    import tkinter.font as _f
    have = set(_f.families(root))
    fam = next((c for c in ("Microsoft YaHei UI", "Microsoft YaHei",
                            "SimHei", "SimSun") if c in have), None)
    if fam is None:
        return None
    for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont",
                 "TkHeadingFont", "TkTooltipFont", "TkCaptionFont",
                 "TkSmallCaptionFont", "TkIconFont"):
        try:
            _f.nametofont(name, root=root).configure(family=fam)
        except Exception:
            pass
    return fam

# ----------------------------------------------------------------- constants

STATE_NAMES = {
    0: "IDLE", 1: "FAULT_STOP", 2: "BRAKSTOP", 3: "NORMSTOP",
    6: "ALIGN", 8: "OL_START", 9: "CL_RUN", 10: "CTRL_RUN",
}
FAULT_NAMES = {0: "-", 1: "OVERCURRENT", 2: "STALL/SYNC"}

HIST = 600                      # samples kept for the plots (~12 s at 50 Hz)
# Names accepted by the firmware's "P <name> <value>" command (see foc_serial.c).
PARAMS = ["rs", "ls", "pllki", "thetaoff", "force", "accel", "starti", "decel",
          "octrip", "ocinst", "stallms", "runms", "fw", "fwgain", "fwmax",
          "obs", "fluxwc", "fluxkp", "fluxki", "psi", "mtpa", "ld", "lq",
          "dt", "dtith", "spdfb", "hoerr", "period"]


def list_com_ports():
    """List serial ports, real USB-UART adapters first.

    Windows enumerates a pile of Bluetooth virtual ports (and the DAPLink VCP)
    ahead of the actual USB-TTL adapter, so the naive first entry is usually a
    dead Bluetooth port.  Sort by usefulness and tag each entry.
    """
    def is_bt(p):
        return "BTHENUM" in (p.hwid or "").upper() or "蓝牙" in (p.description or "")

    def rank(p):
        hw = (p.hwid or "").upper()
        if is_bt(p):
            return 3, p.device            # Bluetooth virtual port: useless here
        if "0D28" in hw:                  # DAPLink VCP: crashes under load
            return 2, p.device
        if "USB" in hw:
            return 0, p.device            # real USB-UART adapter
        return 1, p.device

    out = []
    for p in sorted(list_ports.comports(), key=rank):
        hw = (p.hwid or "").upper()
        if is_bt(p):
            tag = "蓝牙"
        elif "0D28" in hw:
            tag = "DAPLink!勿用"
        elif "USB" in hw:
            tag = "USB ✓"
        else:
            tag = "?"
        out.append((p.device, f"{p.device}  [{tag}] {p.description}"))
    return out


# ------------------------------------------------------------------- serial

class Link(threading.Thread):
    """Background reader; parses $D frames into the sample queue."""

    def __init__(self, port, baud=115200):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.ser = None
        self.rx = queue.Queue()          # parsed telemetry dicts
        self.tx = queue.Queue()          # command strings
        self.events = queue.Queue()      # ('info'|'param'|'error', text)
        self._stop = threading.Event()

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def close(self):
        self._stop.set()
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass

    def send(self, cmd):
        self.tx.put(cmd)

    def run(self):
        buf = b""
        while not self._stop.is_set():
            # ---- transmit
            try:
                while True:
                    cmd = self.tx.get_nowait()
                    self.ser.write((cmd + "\r\n").encode())
            except queue.Empty:
                pass
            # ---- receive
            try:
                chunk = self.ser.read(512)
            except Exception as e:
                self.events.put(("error", f"serial read failed: {e}"))
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if not line:
                    continue
                if line.startswith("$D,"):
                    f = line[3:].split(",")
                    if len(f) >= 9:
                        try:
                            self.rx.put({
                                "t": time.time(),
                                "state": int(float(f[0])),
                                "fault": int(float(f[1])),
                                "spref": float(f[2]),
                                "speed": float(f[3]),
                                "id": float(f[4]),
                                "iq": float(f[5]),
                                "vd": float(f[6]),
                                "vq": float(f[7]),
                                "vbus": float(f[8]),
                            })
                        except ValueError:
                            pass
                else:
                    self.events.put(("info", line))


# ---------------------------------------------------------------- main GUI

class App(tk.Tk):
    def __init__(self, initial_port=None):
        super().__init__()
        self.title("mymotor FOC 上位机  —  STM32G431 sensorless FOC")
        self.geometry("1180x760")
        self.minsize(980, 640)

        self.link = None
        self.hist = {k: deque([0.0] * HIST, maxlen=HIST)
                     for k in ("spref", "speed", "id", "iq", "vbus")}
        self.connected = False
        self.t_last = time.time()
        self._port = None
        self._rx_since = time.time()
        self._warned_nodata = False
        self._last_fault = 0

        self.cjk = apply_cjk_font(self)
        self._build_ui(initial_port)
        self.after(50, self._pump)

    # ---------------------------------------------------------------- layout
    def _build_ui(self, initial_port):
        top = ttk.Frame(self, padding=8)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="串口:").pack(side=tk.LEFT)
        self.cmb_port = ttk.Combobox(top, width=42, state="readonly")
        self.cmb_port.pack(side=tk.LEFT, padx=4)
        self._refresh_ports(initial_port)
        ttk.Button(top, text="刷新", width=6,
                   command=self._refresh_ports).pack(side=tk.LEFT)
        self.btn_conn = ttk.Button(top, text="连接", width=8, command=self._toggle)
        self.btn_conn.pack(side=tk.LEFT, padx=6)

        self.lbl_status = ttk.Label(top, text="未连接", foreground="#a00",
                                    font=("", 10, "bold"))
        self.lbl_status.pack(side=tk.LEFT, padx=12)

        body = ttk.Frame(self)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # ---------------- left: controls
        left = ttk.LabelFrame(body, text="控制", padding=8)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(8, 4), pady=4)

        row = ttk.Frame(left); row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="目标转速 (Hz):", width=14).pack(side=tk.LEFT)
        self.var_spd = tk.DoubleVar(value=40.0)
        self.ent_spd = ttk.Entry(row, textvariable=self.var_spd, width=8)
        self.ent_spd.pack(side=tk.LEFT)
        self.ent_spd.bind("<Return>", lambda e: self._do_run())

        self.scl_spd = ttk.Scale(left, from_=0, to=120, orient=tk.HORIZONTAL,
                                 variable=self.var_spd, length=210,
                                 command=self._on_slide)
        self.scl_spd.pack(fill=tk.X, pady=4)

        row = ttk.Frame(left); row.pack(fill=tk.X, pady=6)
        ttk.Button(row, text="▶ 正转运行", width=12,
                   command=lambda: self._do_run(+1)).pack(side=tk.LEFT, padx=2)
        ttk.Button(row, text="◀ 反转运行", width=12,
                   command=lambda: self._do_run(-1)).pack(side=tk.LEFT, padx=2)
        ttk.Button(left, text="■ 停止 (受控减速)", width=27,
                   command=self._do_stop).pack(fill=tk.X, pady=2)
        ttk.Button(left, text="解除使能 (DIS)", width=27,
                   command=lambda: self._send("DIS")).pack(fill=tk.X, pady=2)
        ttk.Button(left, text="清故障 (STOP)", width=27,
                   command=lambda: self._send("STOP")).pack(fill=tk.X, pady=2)

        # 常用参数
        pf = ttk.LabelFrame(left, text="参数 (P <名> <值>)", padding=6)
        pf.pack(fill=tk.X, pady=(10, 2))
        self.param_vars = {}
        quick = [("runms", "自动停机 ms"), ("force", "闭环切换点 Hz"),
                 ("fw", "弱磁 0/1"), ("mtpa", "MTPA 0/1"), ("pllki", "PLL Ki")]
        for i, (name, label) in enumerate(quick):
            ttk.Label(pf, text=label, width=16).grid(row=i, column=0,
                                                     sticky="w", pady=1)
            v = tk.StringVar(value="")
            self.param_vars[name] = v
            e = ttk.Entry(pf, textvariable=v, width=9)
            e.grid(row=i, column=1, padx=2)
            e.bind("<Return>", lambda ev, n=name: self._set_param(n))
            ttk.Button(pf, text="设置", width=5,
                       command=lambda n=name: self._set_param(n)
                       ).grid(row=i, column=2, padx=2)

        self.chk_log = tk.BooleanVar(value=True)
        ttk.Checkbutton(left, text="显示原始报文", variable=self.chk_log
                        ).pack(anchor="w", pady=(8, 0))

        # ---------------- right: plots + status
        right = ttk.Frame(body)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(4, 8), pady=4)

        self.fig = Figure(figsize=(7.6, 6.4), dpi=100)
        self.ax1 = self.fig.add_subplot(311)
        self.ax2 = self.fig.add_subplot(312)
        self.ax3 = self.fig.add_subplot(313)
        self.fig.subplots_adjust(left=0.09, right=0.97, top=0.96,
                                 bottom=0.06, hspace=0.35)
        for ax, ttl in ((self.ax1, "转速 (Hz)"), (self.ax2, "电流 (A)"),
                        (self.ax3, "母线电压 (V)")):
            ax.set_title(ttl, fontsize=9, loc="left")
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=8)

        (self.ln_ref,) = self.ax1.plot([], [], lw=1.0, ls="--",
                                       color="#888", label="参考")
        (self.ln_spd,) = self.ax1.plot([], [], lw=1.6, color="#1f77b4",
                                       label="实际")
        self.ax1.legend(fontsize=7, loc="upper left", ncol=2)
        (self.ln_id,) = self.ax2.plot([], [], lw=1.3, color="#d62728",
                                      label="Id")
        (self.ln_iq,) = self.ax2.plot([], [], lw=1.3, color="#2ca02c",
                                      label="Iq")
        self.ax2.legend(fontsize=7, loc="upper left", ncol=2)
        (self.ln_vb,) = self.ax3.plot([], [], lw=1.3, color="#9467bd")
        for ax in (self.ax1, self.ax2, self.ax3):
            ax.set_xlim(0, HIST)

        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # status readouts
        sf = ttk.LabelFrame(right, text="状态", padding=6)
        sf.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))
        self.lbl_big = ttk.Label(sf, text="state --   fault --",
                                 font=("Consolas", 13, "bold"))
        self.lbl_big.grid(row=0, column=0, columnspan=6, sticky="w")
        self.readout = {}
        keys = [("spref", "参考Hz"), ("speed", "实际Hz"), ("id", "Id A"),
                ("iq", "Iq A"), ("vd", "Vd V"), ("vq", "Vq V"),
                ("vbus", "Vbus V")]
        for i, (k, lab) in enumerate(keys):
            ttk.Label(sf, text=lab, font=("", 8)).grid(row=1, column=i,
                                                       sticky="w", padx=4)
            v = ttk.Label(sf, text="--", font=("Consolas", 11), width=8)
            v.grid(row=2, column=i, sticky="w", padx=4)
            self.readout[k] = v

        # log
        self.txt = tk.Text(right, height=6, font=("Consolas", 8))
        self.txt.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))
        self.txt.configure(state=tk.DISABLED)

    def _refresh_ports(self, select=None):
        ports = list_com_ports()
        self._ports = ports
        self.cmb_port["values"] = [d for _, d in ports]
        if not ports:
            return
        # Prefer an explicit request, else the first real USB-UART adapter
        # (the list is already sorted with those first).
        want = select or ports[0][0]
        for i, (dev, _) in enumerate(ports):
            if dev == want:
                self.cmb_port.current(i)
                return
        self.cmb_port.current(0)

    def _selected_port(self):
        i = self.cmb_port.current()
        if i < 0 or i >= len(self._ports):
            return None
        return self._ports[i][0]

    # ----------------------------------------------------------- connection
    def _toggle(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self._selected_port()
        if not port:
            messagebox.showwarning("提示", "没有可用串口")
            return
        try:
            self.link = Link(port)
            self.link.open()
            self.link.start()
        except Exception as e:
            messagebox.showerror("连接失败", f"{port}\n{e}")
            self.link = None
            return
        self.connected = True
        self.btn_conn.configure(text="断开")
        self.lbl_status.configure(text=f"已连接 {port} (等待数据...)",
                                  foreground="#c80")
        self._port = port
        self._rx_since = time.time()
        self._warned_nodata = False
        # The firmware auto-stops after g_foc_run_ms (default 30 s) as a safety
        # net for unattended runs.  From the GUI the user expects the motor to
        # keep running until Stop is pressed, so disable that limit on connect.
        # Set a limit again with the 'runms' parameter (ms, 0 = no limit).
        self.link.send("P runms 0")
        self._log(f"[connected to {port}  -  run_ms limited disabled]")
        self._log("[hint] 'runms' 参数可设自动停机时间(ms), 0=不自动停")

    def _disconnect(self):
        if self.link:
            self.link.close()
            self.link = None
        self.connected = False
        self._warned_nodata = False
        self._last_fault = 0
        self.btn_conn.configure(text="连接")
        self.lbl_status.configure(text="未连接", foreground="#a00")
        self._log("[disconnected]")

    # -------------------------------------------------------------- commands
    def _send(self, cmd):
        if not self.connected or not self.link:
            self._log(f"!! 未连接，指令未发送: {cmd}")
            return
        self.link.send(cmd)
        self._log(f">> {cmd}")

    def _do_run(self, direction=+1):
        if not self.connected:
            messagebox.showwarning("未连接",
                                   "请先点右上角的【连接】按钮。\n\n"
                                   "串口应选 COM15（独立的 CP210x 模块）。")
            return
        try:
            hz = abs(float(self.var_spd.get()))
        except (tk.TclError, ValueError):
            hz = 0.0
        if hz < 1.0:
            messagebox.showwarning("转速无效",
                                   f"目标转速是 {hz:.1f} Hz，请先设置（建议 35~100 Hz）。\n\n"
                                   "注意：低于 33 Hz 时闭环无法接管，电机会一直停在 I/F 强拖。")
            return
        self._send(f"RUN {hz * (1 if direction > 0 else -1):.2f}")

    def _do_stop(self):
        self._send("STOP")

    def _on_slide(self, _ev):
        pass

    def _set_param(self, name):
        txt = self.param_vars[name].get().strip()
        if not txt:
            return
        try:
            val = float(txt)
        except ValueError:
            self._log(f"!! {name}: 不是数字")
            return
        self._send(f"P {name} {val:g}")

    def _log(self, s):
        if not self.chk_log.get() and not s.startswith(">>"):
            return
        self.txt.configure(state=tk.NORMAL)
        self.txt.insert(tk.END, s + "\n")
        if int(self.txt.index("end-1c").split(".")[0]) > 300:
            self.txt.delete("1.0", "50.0")
        self.txt.see(tk.END)
        self.txt.configure(state=tk.DISABLED)

    # ------------------------------------------------------------- main loop
    def _pump(self):
        if self.connected and self.link:
            # telemetry
            got = False
            try:
                while True:
                    d = self.link.rx.get_nowait()
                    got = True
                    self._rx_since = time.time()
                    for k in ("spref", "speed"):
                        self.hist[k].append(d[k])
                    self.hist["id"].append(d["id"])
                    self.hist["iq"].append(d["iq"])
                    self.hist["vbus"].append(d["vbus"])
                    self._update_readouts(d)
                if got and not self._warned_nodata:
                    self.lbl_status.configure(text=f"已连接 {self._port}",
                                              foreground="#080")
            except queue.Empty:
                pass
            # no telemetry arriving -> almost always the wrong COM port
            if not self._warned_nodata and (time.time() - self._rx_since) > 4.0:
                self._warned_nodata = True
                self.lbl_status.configure(text=f"{self._port} 无数据 ✗",
                                          foreground="#a00")
                self._log(f"!! {self._port} 收不到遥测 —— 多半选错串口了")
                messagebox.showwarning(
                    "收不到数据",
                    f"{self._port} 已打开但 4 秒内没有任何遥测帧。\n\n"
                    "请断开，在下拉框里选带 [USB ✓] 标记的那个\n"
                    "（你的板子是 Silicon Labs CP210x，即 COM15）。\n\n"
                    "别选 [蓝牙] 或 [DAPLink!勿用]。")
            # events
            try:
                while True:
                    kind, text = self.link.events.get_nowait()
                    self._log(("!! " if kind == "error" else "") + text)
            except queue.Empty:
                pass
            if got:
                self._redraw()
        self.after(50, self._pump)

    def _update_readouts(self, d):
        st = STATE_NAMES.get(d["state"], f"?{d['state']}")
        ft = FAULT_NAMES.get(d["fault"], f"?{d['fault']}")
        colour = "#c00" if d["fault"] else "#060"
        self.lbl_big.configure(text=f"state {st:<10}  fault {ft}", foreground=colour)
        # Make a trip impossible to miss: the fault path decelerates gently, so
        # a protection trip looks like a normal slow stop otherwise.
        if d["fault"] != 0 and d["fault"] != self._last_fault:
            self._last_fault = d["fault"]
            msg = ("过流保护动作\n\n母线电流超过 2.5 A（或瞬时 4 A）。\n"
                   if d["fault"] == 1 else
                   "堵转 / 失步保护动作\n\n电流指令持续顶在限幅上 400 ms，\n"
                   "说明转子跟不上（负载过大、失步或参数不对）。\n")
            self._log(f"!! 保护动作: {ft} (fault={d['fault']})")
            self.bell()
            messagebox.showwarning(f"保护动作 — {ft}",
                                   msg + "\n电机会受控减速停车。")
        if d["fault"] == 0:
            self._last_fault = 0
        for k, lbl in self.readout.items():
            lbl.configure(text=f"{d[k]:.3f}" if k in ("id", "iq") else f"{d[k]:.2f}")

    def _redraw(self):
        x = range(HIST)
        self.ln_ref.set_data(x, list(self.hist["spref"]))
        self.ln_spd.set_data(x, list(self.hist["speed"]))
        self.ln_id.set_data(x, list(self.hist["id"]))
        self.ln_iq.set_data(x, list(self.hist["iq"]))
        self.ln_vb.set_data(x, list(self.hist["vbus"]))

        sp = list(self.hist["speed"]) + list(self.hist["spref"])
        lo, hi = min(sp), max(sp)
        pad = max(5.0, (hi - lo) * 0.15)
        self.ax1.set_ylim(lo - pad, hi + pad)

        for ax, key in ((self.ax2, ("id", "iq")),):
            vals = list(self.hist["id"]) + list(self.hist["iq"])
            m = max(0.5, max(abs(v) for v in vals) * 1.2)
            ax.set_ylim(-m, m)

        vb = list(self.hist["vbus"])
        self.ax3.set_ylim(min(vb) - 1.0, max(vb) + 1.0)
        self.canvas.draw_idle()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    app = App(port)
    app.mainloop()


if __name__ == "__main__":
    main()
