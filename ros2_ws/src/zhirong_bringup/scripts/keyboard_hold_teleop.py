#!/usr/bin/env python3

import signal
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


PUBLISH_INTERVAL_MS = 50
RELEASE_DEBOUNCE_MS = 70


class HoldToRunTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_hold_teleop")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.held_keys = set()
        self.pending_releases = {}
        self.closing = False

        self.root = tk.Tk()
        self.root.title("Zhirong Hold-to-Run Control")
        self.root.geometry("520x430")
        self.root.minsize(520, 430)
        self.root.configure(bg="#172033")

        self.forward_speed = tk.DoubleVar(value=0.35)
        self.reverse_speed = tk.DoubleVar(value=0.25)
        self.turn_speed = tk.DoubleVar(value=1.00)
        self.state_text = tk.StringVar(value="STOPPED | waiting for key")
        self.speed_text = tk.StringVar(value="linear.x=+0.00 m/s   angular.z=+0.00 rad/s")

        self._build_window()
        self.root.bind_all("<KeyPress>", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)
        self.root.bind("<FocusOut>", self._on_focus_out)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(PUBLISH_INTERVAL_MS, self._update)
        self.root.after(250, self.root.focus_force)

    def _build_window(self):
        title = tk.Label(
            self.root,
            text="4-Wheel Robot | Hold to Move",
            font=("Sans", 18, "bold"),
            fg="#f8fafc",
            bg="#172033",
        )
        title.pack(pady=(18, 4))

        safety = tk.Label(
            self.root,
            text="W Forward   S Reverse   A Left   D Right   Space STOP\n"
            "Release the key or leave this window to stop",
            font=("Sans", 12),
            fg="#b8c7e0",
            bg="#172033",
            justify="center",
        )
        safety.pack(pady=(0, 12))

        key_frame = tk.Frame(self.root, bg="#172033")
        key_frame.pack()
        self.key_labels = {}
        for key, row, column, label in (
            ("w", 0, 1, "W\nForward"),
            ("a", 1, 0, "A\nLeft"),
            ("s", 1, 1, "S\nReverse"),
            ("d", 1, 2, "D\nRight"),
        ):
            key_label = tk.Label(
                key_frame,
                text=label,
                width=8,
                height=3,
                font=("Sans", 12, "bold"),
                fg="#e2e8f0",
                bg="#334155",
                relief="raised",
                borderwidth=2,
            )
            key_label.grid(row=row, column=column, padx=7, pady=5)
            self.key_labels[key] = key_label

        speed_frame = tk.Frame(self.root, bg="#172033")
        speed_frame.pack(fill="x", padx=35, pady=(12, 4))
        self._add_scale(
            speed_frame,
            "Forward",
            self.forward_speed,
            0.10,
            0.80,
            0,
        )
        self._add_scale(
            speed_frame,
            "Reverse",
            self.reverse_speed,
            0.10,
            0.50,
            1,
        )
        self._add_scale(
            speed_frame,
            "Turning",
            self.turn_speed,
            0.30,
            2.00,
            2,
        )

        state = tk.Label(
            self.root,
            textvariable=self.state_text,
            font=("Sans", 13, "bold"),
            fg="#5eead4",
            bg="#172033",
        )
        state.pack(pady=(8, 2))

        speed = tk.Label(
            self.root,
            textvariable=self.speed_text,
            font=("Monospace", 11),
            fg="#e2e8f0",
            bg="#172033",
        )
        speed.pack()

    def _add_scale(self, parent, text, variable, minimum, maximum, row):
        label = tk.Label(
            parent,
            text=text,
            width=8,
            anchor="w",
            fg="#e2e8f0",
            bg="#172033",
        )
        label.grid(row=row, column=0, sticky="w")

        scale = tk.Scale(
            parent,
            from_=minimum,
            to=maximum,
            resolution=0.05,
            orient="horizontal",
            variable=variable,
            showvalue=True,
            length=330,
            fg="#e2e8f0",
            bg="#172033",
            troughcolor="#475569",
            highlightthickness=0,
        )
        scale.grid(row=row, column=1, sticky="ew")

    def _on_key_press(self, event):
        key = event.keysym.lower()
        if key == "space":
            self.stop_motion()
            return
        if key not in self.key_labels:
            return

        pending = self.pending_releases.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self.held_keys.add(key)

    def _on_key_release(self, event):
        key = event.keysym.lower()
        if key not in self.key_labels:
            return

        pending = self.pending_releases.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self.pending_releases[key] = self.root.after(
            RELEASE_DEBOUNCE_MS,
            lambda released_key=key: self._finish_key_release(released_key),
        )

    def _finish_key_release(self, key):
        self.pending_releases.pop(key, None)
        self.held_keys.discard(key)
        self.publish_command()

    def _on_focus_out(self, _event):
        self.stop_motion()

    def command(self):
        forward = "w" in self.held_keys
        reverse = "s" in self.held_keys
        left = "a" in self.held_keys
        right = "d" in self.held_keys

        if forward == reverse:
            linear = 0.0
        elif forward:
            linear = self.forward_speed.get()
        else:
            linear = -self.reverse_speed.get()

        if left == right:
            angular = 0.0
        elif left:
            angular = self.turn_speed.get()
        else:
            angular = -self.turn_speed.get()

        return linear, angular

    def publish_command(self):
        linear, angular = self.command()
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.publisher.publish(message)
        return linear, angular

    def stop_motion(self):
        self.held_keys.clear()
        for callback_id in self.pending_releases.values():
            self.root.after_cancel(callback_id)
        self.pending_releases.clear()
        for _ in range(3):
            self.publish_command()

    def _update_labels(self, linear, angular):
        for key, label in self.key_labels.items():
            label.configure(
                bg="#0f766e" if key in self.held_keys else "#334155",
                relief="sunken" if key in self.held_keys else "raised",
            )

        movements = []
        if linear > 0:
            movements.append("FORWARD")
        elif linear < 0:
            movements.append("REVERSE")
        if angular > 0:
            movements.append("LEFT")
        elif angular < 0:
            movements.append("RIGHT")

        self.state_text.set(
            " + ".join(movements) if movements else "STOPPED | waiting for key"
        )
        self.speed_text.set(
            f"linear.x={linear:+.2f} m/s   angular.z={angular:+.2f} rad/s"
        )

    def _update(self):
        if self.closing:
            return

        rclpy.spin_once(self, timeout_sec=0.0)
        linear, angular = self.publish_command()
        self._update_labels(linear, angular)
        self.root.after(PUBLISH_INTERVAL_MS, self._update)

    def run(self):
        self.root.mainloop()

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.stop_motion()
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()


def main():
    rclpy.init()
    node = HoldToRunTeleop()

    def request_close(_signal_number, _frame):
        node.root.after(0, node.close)

    signal.signal(signal.SIGINT, request_close)
    signal.signal(signal.SIGTERM, request_close)

    try:
        node.run()
    except KeyboardInterrupt:
        node.close()


if __name__ == "__main__":
    main()
