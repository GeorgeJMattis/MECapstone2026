"""Three-motor stepper control panel.

Run with ``python3 stepper_gui.py`` after installing ``requirements.txt``.
"""
import math
import tkinter as tk
from tkinter import ttk

import serial
import serial.tools.list_ports

BAUD = 115200
BG, FG, ACCENT, DIM = "#1e1f24", "#e6e6e6", "#4fa3ff", "#3a3c44"
AXES = ("A", "B", "C")


class MotorPanel:
    def __init__(self, app, parent, axis):
        self.app = app
        self.axis = axis
        self.turns = 0.0
        self.last_sent = {}
        self.drag_angle = None

        body = ttk.Frame(parent, padding=8)
        body.pack(fill="both", expand=True)
        left = ttk.Frame(body)
        left.pack(side="left", fill="both", expand=True)

        modes = ttk.Frame(left)
        modes.pack(fill="x")
        self.mode = tk.StringVar(value="pos")
        ttk.Radiobutton(modes, text="Position (wheel)", value="pos", variable=self.mode,
                        command=self.mode_changed).pack(side="left")
        ttk.Radiobutton(modes, text="Constant rotation", value="vel", variable=self.mode,
                        command=self.mode_changed).pack(side="left", padx=12)

        self.canvas = tk.Canvas(left, width=320, height=320, bg=BG, highlightthickness=0)
        self.canvas.pack(pady=6)
        self.canvas.bind("<ButtonPress-1>", self.wheel_press)
        self.canvas.bind("<B1-Motion>", self.wheel_drag)
        self.canvas.bind("<ButtonRelease-1>", lambda _event: setattr(self, "drag_angle", None))
        self.canvas.bind("<MouseWheel>", self.wheel_scroll)
        self.pos_label = ttk.Label(left, text="", font=("Consolas", 12))
        self.pos_label.pack()

        buttons = ttk.Frame(left)
        buttons.pack(pady=4)
        ttk.Button(buttons, text="Zero here", command=self.zero).pack(side="left", padx=3)
        ttk.Button(buttons, text="Go to 0", command=lambda: self.set_turns(0)).pack(side="left", padx=3)
        ttk.Button(buttons, text="+1 turn", command=lambda: self.set_turns(self.turns + 1)).pack(
            side="left", padx=3)
        ttk.Button(buttons, text="−1 turn", command=lambda: self.set_turns(self.turns - 1)).pack(
            side="left", padx=3)

        speed = ttk.LabelFrame(left, text="Constant rotation speed (RPM)", padding=6)
        speed.pack(fill="x", pady=6)
        self.rpm = tk.DoubleVar(value=0)
        tk.Scale(speed, from_=-300, to=300, orient="horizontal", resolution=1,
                 variable=self.rpm, command=lambda _value: self.send_velocity(),
                 length=300, bg=BG, fg=FG, troughcolor=DIM, highlightthickness=0,
                 activebackground=ACCENT).pack(fill="x")
        ttk.Button(speed, text="STOP", command=self.stop).pack(pady=(4, 0))

        tuning = ttk.LabelFrame(body, text=f"Motor {axis} tuning", padding=8)
        tuning.pack(side="left", fill="y", padx=(12, 0))
        self.kp = self.fader(tuning, "Snappy", 5, 2000, 150, 5)
        self.zeta = self.fader(tuning, "Damping", 0.05, 2.0, 1.0, 0.05)
        self.vmax = self.fader(tuning, "Max RPM", 10, 1000, 300, 10)
        self.amax = self.fader(tuning, "Accel", 50, 20000, 3000, 50)
        ttk.Label(tuning, text="Damping < 1 = bouncy\nDamping ≈ 1 = crisp\nAccel in RPM/s",
                  foreground="#999").grid(row=2, column=0, columnspan=4, pady=(8, 0))
        self.draw_wheel()

    def fader(self, parent, name, lo, hi, initial, resolution):
        column = len(parent.grid_slaves(row=0))
        var = tk.DoubleVar(value=initial)
        ttk.Label(parent, text=name).grid(row=0, column=column, padx=6)
        tk.Scale(parent, from_=hi, to=lo, resolution=resolution, variable=var, length=260,
                 width=22, command=lambda _value: self.send_tuning(), bg=BG, fg=FG,
                 troughcolor=DIM, highlightthickness=0, activebackground=ACCENT,
                 sliderlength=18).grid(row=1, column=column, padx=6)
        return var

    def draw_wheel(self):
        canvas = self.canvas
        canvas.delete("all")
        cx, cy, radius = 160, 160, 130
        canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius,
                           outline=DIM, width=14)
        for index in range(36):
            angle = math.radians(index * 10)
            inner = radius - 22 if index % 9 == 0 else radius - 14
            canvas.create_line(cx + inner * math.sin(angle), cy - inner * math.cos(angle),
                               cx + (radius - 8) * math.sin(angle),
                               cy - (radius - 8) * math.cos(angle), fill="#666")
        fraction = self.turns % 1.0
        if fraction > 1e-6:
            canvas.create_arc(cx - radius, cy - radius, cx + radius, cy + radius,
                              start=90, extent=-fraction * 360, style="arc",
                              outline=ACCENT, width=14)
        angle = math.radians(self.turns * 360)
        canvas.create_line(cx, cy, cx + (radius - 30) * math.sin(angle),
                           cy - (radius - 30) * math.cos(angle), fill=ACCENT,
                           width=5, capstyle="round")
        canvas.create_oval(cx - 10, cy - 10, cx + 10, cy + 10, fill=ACCENT, outline="")
        enabled = self.mode.get() == "pos"
        canvas.create_text(cx, cy + 50,
                           text=f"{self.turns * 360:+.1f}°" if enabled else "rotation mode",
                           fill=FG if enabled else "#777", font=("Consolas", 14))
        self.pos_label.config(
            text=f"turns {self.turns:+.3f}   steps {self.target_steps():+d}")

    def angle_at(self, event):
        return math.atan2(event.x - 160, -(event.y - 160)) / (2 * math.pi)

    def wheel_press(self, event):
        if self.mode.get() == "pos":
            self.drag_angle = self.angle_at(event)

    def wheel_drag(self, event):
        if self.drag_angle is None:
            return
        angle = self.angle_at(event)
        delta = angle - self.drag_angle
        delta -= round(delta)
        self.drag_angle = angle
        self.set_turns(self.turns + delta)

    def wheel_scroll(self, event):
        if self.mode.get() == "pos":
            delta = event.delta / 120 if abs(event.delta) >= 120 else event.delta
            self.set_turns(self.turns + delta / 72)

    def set_turns(self, turns):
        if self.mode.get() != "pos":
            return
        self.turns = turns
        self.draw_wheel()

    def target_steps(self):
        return int(round(self.turns * self.app.steps_per_rev()))

    def send(self, command, force=False):
        key = command[0]
        if not force and self.last_sent.get(key) == command:
            return
        if self.app.send_line(f"{self.axis} {command}"):
            self.last_sent[key] = command

    def rpm_to_sps(self, rpm):
        return rpm * self.app.steps_per_rev() / 60.0

    def send_tuning(self):
        self.send(f"G {self.kp.get():.1f} {self.zeta.get():.2f} "
                  f"{self.rpm_to_sps(self.vmax.get()):.1f} "
                  f"{self.rpm_to_sps(self.amax.get()):.1f}")

    def send_velocity(self):
        if self.mode.get() == "vel":
            self.send(f"V {self.rpm_to_sps(self.rpm.get()):.2f}")

    def stop(self):
        self.rpm.set(0)
        self.mode.set("vel")
        self.send("S", force=True)
        self.draw_wheel()

    def zero(self):
        self.send("Z", force=True)
        self.turns = 0
        self.last_sent.pop("P", None)
        self.draw_wheel()

    def mode_changed(self):
        if self.mode.get() == "vel":
            self.last_sent.pop("V", None)
            self.send_tuning()
            self.send_velocity()
        else:
            self.rpm.set(0)
            self.zero()
            self.send("P 0", force=True)
        self.draw_wheel()


class StepperGUI:
    def __init__(self, root):
        self.root = root
        self.ser = None

        root.title("Three-Motor Stepper Control")
        root.configure(bg=BG)
        root.geometry("900x550")
        style = ttk.Style()
        style.theme_use("clam")
        for widget in ("TFrame", "TLabel", "TRadiobutton", "TLabelframe", "TLabelframe.Label"):
            style.configure(widget, background=BG, foreground=FG)
        style.configure("TButton", background=DIM, foreground=FG)

        connection = ttk.Frame(root, padding=8)
        connection.pack(fill="x")
        ttk.Label(connection, text="Serial port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(connection, textvariable=self.port_var, width=34,
                                     state="readonly")
        self.port_box.pack(side="left", padx=4)
        ttk.Button(connection, text="↻", width=3, command=self.refresh_ports).pack(side="left")
        self.conn_btn = ttk.Button(connection, text="Connect", command=self.toggle_connect)
        self.conn_btn.pack(side="left", padx=4)
        ttk.Label(connection, text="Steps/rev").pack(side="left", padx=(14, 2))
        self.spr_var = tk.StringVar(value="1600")
        ttk.Entry(connection, textvariable=self.spr_var, width=7).pack(side="left")
        self.status = ttk.Label(connection, text="disconnected", foreground="#ff7070")
        self.status.pack(side="right")

        group = ttk.LabelFrame(root, text="Synchronized group controls", padding=8)
        group.pack(fill="x", padx=8)
        ttk.Button(group, text="Move all to each tab's target",
                   command=self.sync_positions).pack(side="left", padx=3)
        ttk.Label(group, text="Same target (turns)").pack(side="left", padx=(12, 3))
        self.broadcast_var = tk.StringVar(value="0")
        ttk.Entry(group, textvariable=self.broadcast_var, width=8).pack(side="left")
        ttk.Button(group, text="Broadcast target to all",
                   command=self.broadcast_position).pack(side="left", padx=3)
        ttk.Button(group, text="Rotate all at each tab's speed",
                   command=self.sync_rotation).pack(side="left", padx=3)
        ttk.Button(group, text="STOP ALL", command=self.stop_all).pack(side="right", padx=3)

        self.tabs = ttk.Notebook(root)
        self.tabs.pack(fill="both", expand=True, padx=8, pady=8)
        self.panels = []
        for axis in AXES:
            frame = ttk.Frame(self.tabs)
            self.tabs.add(frame, text=f"Motor {axis}")
            self.panels.append(MotorPanel(self, frame, axis))

        self.refresh_ports()
        self.root.after(30, self.tick)

    def steps_per_rev(self):
        try:
            value = float(self.spr_var.get())
            if value > 0:
                return value
        except ValueError:
            pass
        return 1600.0

    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        names = [f"{port.device} — {port.description}" for port in ports]
        self.port_box["values"] = names
        pick = next((name for name, port in zip(names, ports)
                     if "CH340" in port.description or port.vid == 0x1A86),
                    next((name for name, port in zip(names, ports)
                          if name.startswith("/dev/cu.")), names[0] if names else ""))
        self.port_var.set(pick)

    def toggle_connect(self):
        if self.ser:
            self.ser.close()
            self.ser = None
            self.conn_btn.config(text="Connect")
            self.status.config(text="disconnected", foreground="#ff7070")
            return
        device = self.port_var.get().split(" — ", 1)[0]
        if not device:
            self.status.config(text="No serial port selected", foreground="#ff7070")
            return
        try:
            self.ser = serial.Serial(device, BAUD, timeout=0, write_timeout=0.2)
        except serial.SerialException as error:
            self.status.config(text=str(error)[:60], foreground="#ff7070")
            return
        self.conn_btn.config(text="Disconnect")
        self.status.config(text=f"connected {device} (board resetting…)",
                           foreground="#70ff90")
        for panel in self.panels:
            panel.last_sent.clear()
        self.root.after(2000, self.on_ready)

    def on_ready(self):
        if not self.ser:
            return
        self.status.config(text=f"connected {self.ser.port}", foreground="#70ff90")
        for panel in self.panels:
            panel.turns = 0
            panel.mode.set("pos")
            panel.rpm.set(0)
            panel.draw_wheel()
            panel.send_tuning()
        self.send_line("M P 0 0 0")

    def send_line(self, line):
        if not self.ser:
            return False
        try:
            self.ser.write((line + "\n").encode("ascii"))
        except (serial.SerialException, serial.SerialTimeoutException) as error:
            self.status.config(text=f"error: {error}"[:60], foreground="#ff7070")
            return False
        return True

    def sync_positions(self):
        targets = [panel.target_steps() for panel in self.panels]
        for panel in self.panels:
            panel.mode.set("pos")
            panel.rpm.set(0)
            panel.last_sent.pop("P", None)
            panel.draw_wheel()
        self.send_line(f"M P {targets[0]} {targets[1]} {targets[2]}")

    def broadcast_position(self):
        try:
            turns = float(self.broadcast_var.get())
        except ValueError:
            self.status.config(text="Target must be a number", foreground="#ff7070")
            return
        steps = [int(round(turns * self.steps_per_rev()))] * 3
        for panel in self.panels:
            panel.turns = turns
            panel.mode.set("pos")
            panel.rpm.set(0)
            panel.last_sent.pop("P", None)
            panel.draw_wheel()
        self.send_line(f"M P {steps[0]} {steps[1]} {steps[2]}")

    def sync_rotation(self):
        speeds = []
        for panel in self.panels:
            panel.mode.set("vel")
            panel.last_sent.pop("V", None)
            panel.send_tuning()
            speeds.append(panel.rpm_to_sps(panel.rpm.get()))
            panel.draw_wheel()
        self.send_line(f"M V {speeds[0]:.2f} {speeds[1]:.2f} {speeds[2]:.2f}")

    def stop_all(self):
        for panel in self.panels:
            panel.rpm.set(0)
            panel.mode.set("vel")
            panel.draw_wheel()
        self.send_line("M S")

    def tick(self):
        for panel in self.panels:
            if panel.mode.get() == "pos":
                panel.send(f"P {panel.target_steps()}")
        self.root.after(30, self.tick)


if __name__ == "__main__":
    root = tk.Tk()
    StepperGUI(root)
    root.mainloop()
