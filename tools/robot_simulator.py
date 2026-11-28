"""Windows desktop simulator. Run with python tools/robot_simulator.py."""
import struct
import time
import tkinter as tk
from datetime import datetime
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

import serial
from serial.tools import list_ports

import dm_usbcan
import simulator_protocols as protocol
from referee_simulator import MESSAGES


ROLES = ("NUC", "NX", "裁判系统")
DECISION = "0x401  自决策值"
DRIVE = "0x402  底盘速度与折叠请求"
FIELDS = "0x404  字段说明（离线）"
GROUP = "整组 · 10 条裁判报文"


def icon(role):
    image = tk.PhotoImage(width=30, height=26)
    color = "#196b80"
    def rect(x, y, w, h, ink=color):
        image.put(ink, to=(x, y, x+w, y+h))
    if role == "NUC":
        rect(2, 2, 26, 17)
        rect(5, 5, 20, 10, "#e9f5f7")
        rect(12, 19, 6, 3)
        rect(7, 22, 16, 2)
    elif role == "NX":
        rect(6, 5, 17, 16)
        rect(10, 9, 9, 8, "#e9f5f7")
        for x in (8, 13, 18):
            rect(x, 2, 2, 3)
            rect(x, 21, 2, 3)
        for y in (7, 12, 17):
            rect(3, y, 3, 2)
            rect(23, y, 3, 2)
    else:
        rect(6, 3, 18, 21)
        rect(9, 7, 12, 14, "#e9f5f7")
        rect(11, 1, 8, 5)
        for y in (10, 14, 18):
            rect(11, y, 8, 1)
    return image


class Simulator:
    def __init__(self, root):
        self.root = root
        root.title("哨兵通信调试台")
        root.geometry("870x680")
        root.minsize(740, 610)
        self.links = {}
        self.selectors = {}
        self.connect_buttons = {}
        self.rx = dm_usbcan.Receiver()
        self.job = None
        self.sequence = 0
        self.started = time.monotonic()
        self.status_time = None
        self.last_rx_log = {}
        self.next_due = {}
        self.role = ""
        self.command = tk.StringVar()
        self.interval = tk.StringVar(value="50")
        self.feedback = tk.StringVar(value="0x505　等待回传")
        self.activity = tk.StringVar(value="已停止")
        self.opts = {}
        style = ttk.Style()
        style.configure("TNotebook.Tab", padding=(18, 8))
        main = ttk.Frame(root, padding=14)
        main.pack(fill="both", expand=True)
        links = ttk.LabelFrame(main, text="设备连接", padding=8)
        links.pack(fill="x")
        for col, kind in enumerate(("CAN", "UART")):
            block = ttk.Frame(links)
            block.grid(row=0, column=col, sticky="w", padx=5)
            ttk.Label(block, text=kind).pack(side="left", padx=4)
            box = ttk.Combobox(block, state="readonly", width=11)
            box.pack(side="left")
            self.selectors[kind] = box
            btn = ttk.Button(block, text="连接", width=6, command=lambda k=kind: self.toggle(k))
            btn.pack(side="left", padx=4)
            self.connect_buttons[kind] = btn
        ttk.Button(links, text="刷新端口", command=self.refresh).grid(row=0, column=2, padx=8)
        self.tabs = ttk.Notebook(main)
        self.tabs.pack(fill="x", pady=(12, 0))
        self.icons = [icon(role) for role in ROLES]
        for role, picture in zip(ROLES, self.icons):
            self.tabs.add(ttk.Frame(self.tabs), text=role, image=picture, compound="left")
        self.tabs.bind("<<NotebookTabChanged>>", self.change_role)
        self.panel = ttk.Frame(main, padding=(8, 9))
        self.panel.pack(fill="x")
        ttk.Label(self.panel, text="发送指令").grid(row=0, column=0, padx=(0, 10))
        self.menu = ttk.Combobox(self.panel, state="readonly", width=36, textvariable=self.command)
        self.menu.grid(row=0, column=1, sticky="w")
        self.menu.bind("<<ComboboxSelected>>", self.change_command)
        self.form = ttk.Frame(main, padding=(8, 0))
        self.form.pack(fill="x")
        actions = ttk.Frame(main, padding=(8, 12))
        actions.pack(fill="x")
        ttk.Label(actions, text="周期 (ms)").pack(side="left")
        self.period = ttk.Entry(actions, width=7, textvariable=self.interval)
        self.period.pack(side="left", padx=5)
        self.once = ttk.Button(actions, text="单次发送", command=lambda: self.start(False))
        self.repeat = ttk.Button(actions, text="周期发送", command=lambda: self.start(True))
        for btn in (self.once, self.repeat):
            btn.pack(side="left", padx=4)
        ttk.Button(actions, text="停止发送", command=self.stop).pack(side="left", padx=4)
        ttk.Label(actions, textvariable=self.activity).pack(side="right")
        ttk.Label(main, textvariable=self.feedback, foreground="#176477").pack(anchor="w", pady=5)
        logbar = ttk.Frame(main)
        logbar.pack(fill="x")
        ttk.Label(logbar, text="收发记录").pack(side="left")
        ttk.Button(logbar, text="清空", command=self.clear_log).pack(side="right")
        self.log = ScrolledText(main, height=12, font=("Consolas", 10), wrap="word", state="disabled")
        self.log.pack(fill="both", expand=True, pady=(5, 0))
        self.refresh()
        self.change_role()
        self.poll_job = root.after(20, self.poll)
        root.protocol("WM_DELETE_WINDOW", self.close)

    def note(self, message):
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log.configure(state="normal")
        self.log.insert("end", f"{stamp}  {message}\n")
        if int(self.log.index("end-1c").split(".")[0]) > 650:
            self.log.delete("1.0", "101.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def clear_log(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def refresh(self):
        devices = [p.device for p in list_ports.comports()]
        for kind, box in self.selectors.items():
            box.configure(values=devices)
            if kind not in self.links and box.get() not in devices:
                box.set("")

    def disconnect(self, kind):
        self.stop()
        port = self.links.pop(kind, None)
        if port is not None:
            port.close()
        self.connect_buttons[kind].configure(text="连接")
        self.selectors[kind].configure(state="readonly")
        if kind == "CAN":
            self.status_time = None
            self.rx = dm_usbcan.Receiver()
            self.feedback.set("0x505　等待回传")

    def toggle(self, kind):
        if kind in self.links:
            self.disconnect(kind)
            return
        port = self.selectors[kind].get()
        if not port or port not in {p.device for p in list_ports.comports()}:
            self.note("请刷新并选择本次接线对应的 COM 口")
            return
        if any(link.port == port for link in self.links.values()):
            self.note("CAN 和 UART 需要分别选择对应的 DM Tool")
            return
        try:
            baud = 921600 if kind == "CAN" else 115200
            self.links[kind] = serial.Serial(port, baud, timeout=0, write_timeout=0.15)
        except (OSError, ValueError) as error:
            self.note(f"连接失败　{error}")
            return
        self.selectors[kind].configure(state="disabled")
        self.connect_buttons[kind].configure(text="断开")
        self.note(f"已连接 {kind} · {port} · 串口 {baud}")

    def change_role(self, event=None):
        role = ROLES[self.tabs.index("current")]
        if role == self.role:
            return
        self.role = role
        choices = {"NUC": (DRIVE, DECISION), "NX": (FIELDS,),
                   "裁判系统": (GROUP, *protocol.REF_COMMANDS)}[role]
        self.menu.configure(values=choices)
        self.menu.current(0)
        self.change_command()

    def field(self, key, label, value, choices=None):
        row = len(self.opts)
        var = tk.StringVar(value=value)
        self.opts[key] = var
        ttk.Label(self.form, text=label, width=20).grid(row=row, column=0, sticky="w", pady=2)
        if choices:
            box = ttk.Combobox(self.form, textvariable=var, values=choices, state="readonly", width=20)
        else:
            box = ttk.Entry(self.form, textvariable=var, width=23)
        box.grid(row=row, column=1, sticky="w", pady=2)

    def change_command(self, event=None):
        self.stop()
        for child in self.form.winfo_children():
            child.destroy()
        self.opts.clear()
        cmd = self.command.get()
        if cmd == DRIVE:
            self.field("vx", "世界系 vx (m/s)", "0")
            self.field("vy", "世界系 vy (m/s)", "0")
            self.field("fold", "机构目标", "折叠", ("折叠", "直立"))
            self.field("spin", "小陀螺", "关", ("关", "开"))
            self.field("cap", "超电请求", "关", ("关", "开"))
        elif cmd == DECISION:
            self.field("decision", "uint32 自决策值", "0")
        elif self.role == "裁判系统":
            self.field("robot", "哨兵编号", "7", ("7", "107"))
            self.field("mode", "串口数据模式", "正常", tuple(protocol.MODES))
        else:
            msg = "字段说明　flags 包含开火与扫描请求，另外两个字段为 yaw、pitch。"
            ttk.Label(self.form, text=msg).pack(anchor="w")
        state = "disabled" if cmd == FIELDS else "normal"
        self.once.configure(state=state)
        self.repeat.configure(state=state)
        self.period.configure(state="disabled" if cmd in (GROUP, FIELDS) else "normal")

    def stop(self):
        if self.job is not None:
            self.root.after_cancel(self.job)
            self.job = None
        self.activity.set("已停止")

    def start(self, periodic):
        self.stop()
        cmd = self.command.get()
        if self.role == "NX":
            return
        kind = "UART" if self.role == "裁判系统" else "CAN"
        if kind not in self.links:
            self.note(f"请先连接 {kind} 接口")
            return
        opt = {key: var.get() for key, var in self.opts.items()}
        try:
            period = 10 if cmd == GROUP or not periodic else int(self.interval.get())
            if not 10 <= period <= 60000:
                raise ValueError("发送周期须为 10 至 60000 ms")
            if cmd == DRIVE:
                payload = protocol.chassis(opt["vx"], opt["vy"], opt["fold"] == "折叠",
                                           opt["spin"] == "开", opt["cap"] == "开")
                can_frame = (0x402, payload)
            elif cmd == DECISION:
                can_frame = (0x401, protocol.decision(opt["decision"]))
            elif kind == "CAN":
                raise ValueError("未知的 CAN 发送指令")
        except (ValueError, OverflowError) as error:
            self.note(f"参数错误　{error}")
            return
        self.next_due.clear()
        def cycle():
            self.job = None
            try:
                if kind == "CAN":
                    ident, data = can_frame
                    self.write(kind, dm_usbcan.packet(ident, data))
                    self.note(f"TX {self.role}  0x{ident:03X}  [{len(data)}]  {data.hex(' ')}")
                else:
                    self.send_ref(cmd, opt, periodic)
            except (OSError, ValueError) as error:
                self.note(f"发送失败　{error}")
                self.disconnect(kind)
                return
            if periodic:
                self.activity.set("周期发送中")
                self.job = self.root.after(period, cycle)
            else:
                self.activity.set("已单次发送")
        cycle()

    def write(self, kind, data):
        if self.links[kind].write(data) != len(data):
            raise OSError("串口写入不完整")

    def send_ref(self, cmd, opt, periodic):
        now = time.monotonic()
        commands = []
        if cmd == GROUP:
            for ident, _, hz, _ in MESSAGES:
                if not periodic or now >= self.next_due.get(ident, 0):
                    commands.append(ident)
                    self.next_due[ident] = now + 1 / hz
        else:
            commands.append(protocol.REF_COMMANDS[cmd])
        mode = protocol.MODES[opt["mode"]]
        frames = protocol.referee_frames(commands, now - self.started, int(opt["robot"]), self.sequence, mode)
        if frames and mode == "burst":
            self.write("UART", b"".join(data for _, data in frames))
        for ident, data in frames:
            if mode == "split":
                self.write("UART", data[:5])
                time.sleep(0.005)
                self.write("UART", data[5:])
            elif mode != "burst":
                self.write("UART", data)
            self.note(f"TX 裁判  0x{ident:04X}  {data.hex(' ')}")
        self.sequence = (self.sequence + len(frames)) & 255

    def poll(self):
        link = self.links.get("CAN")
        now = time.monotonic()
        if link:
            try:
                chunk = link.read(min(link.in_waiting, 8192))
                for ident, data in self.rx.feed(chunk):
                    if ident not in (0x501, 0x504, 0x505):
                        continue
                    if ident == 0x505 and len(data) == 1:
                        self.feedback.set("0x505　" + protocol.fold_status(data))
                        self.status_time = now
                    previous = self.last_rx_log.get(ident)
                    if ident != 0x501 and previous and previous[0] == data and now - previous[1] < 0.5:
                        continue
                    self.last_rx_log[ident] = data, now
                    label = ""
                    if ident == 0x504 and len(data) == 8:
                        label = "  q=" + str(tuple(v / 10000 for v in struct.unpack("<4h", data)))
                    self.note(f"RX CAN  0x{ident:03X}  [{len(data)}]  {data.hex(' ')}{label}")
            except (OSError, ValueError) as error:
                self.note(f"接收失败　{error}")
                self.disconnect("CAN")
        if self.status_time is not None and now - self.status_time >= 0.2:
            self.feedback.set("0x505　回传已过期")
            self.status_time = None
        self.poll_job = self.root.after(20, self.poll)

    def close(self):
        self.root.after_cancel(self.poll_job)
        for kind in tuple(self.links):
            self.disconnect(kind)
        self.stop()
        self.root.destroy()


if __name__ == "__main__":
    window = tk.Tk()
    Simulator(window)
    window.mainloop()
