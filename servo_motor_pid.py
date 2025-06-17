import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
import numpy as np
from collections import deque

# ==============================================================================
# PID and RobotArm classes 
# ==============================================================================
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits=(-100, 100)):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self._integral = 0
        self._last_error = 0

    def update(self, process_variable, dt):
        if dt <= 0: return 0
        error = self.setpoint - process_variable
        self._integral += error * dt
        derivative = (error - self._last_error) / dt
        p_term = self.Kp * error
        i_term = self.Ki * self._integral
        d_term = self.Kd * derivative
        output = p_term + i_term + d_term
        if self.output_limits is not None:
            output = np.clip(output, self.output_limits[0], self.output_limits[1])
        self._last_error = error
        return output

class RobotArm:
    def __init__(self, initial_angle=0.0):
        self.mass = 1.0; self.total_length = 1.0; self.length = 0.5; self.g = 9.81
        self.inertia = (1/3) * self.mass * self.total_length**2
        self.angle = initial_angle; self.velocity = 0.0
        self.damping = 0.1; self.motor_gain = 5.0

    def update(self, motor_output, dt):
        motor_torque = self.motor_gain * (motor_output / 100.0)
        gravity_torque = self.mass * self.g * self.length * np.cos(np.radians(self.angle))
        damping_torque = self.damping * np.radians(self.velocity)
        net_torque = motor_torque - gravity_torque - damping_torque
        angular_acceleration = net_torque / self.inertia
        self.velocity += np.degrees(angular_acceleration) * dt
        self.angle += self.velocity * dt
        if self.angle >= 180 or self.angle <= 0:
            self.velocity = 0
        self.angle = np.clip(self.angle, 0, 180)
        return self.angle

# ==============================================================================
# Main Interactive Application 
# ==============================================================================
class PIDSimulatorApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("High-Performance PID Simulator")

        self.max_data_points = 200
        self.times = deque(maxlen=self.max_data_points)
        self.angles = deque(maxlen=self.max_data_points)
        self.setpoints = deque(maxlen=self.max_data_points)
        self.outputs = deque(maxlen=self.max_data_points)
        self.dt = 0.02  # Use a smaller dt for smoother animation
        self.last_time = 0

        controls_frame = ttk.Frame(self, padding="10")
        controls_frame.grid(row=0, column=0, sticky="nsew")
        plot_frame = ttk.Frame(self, padding="10")
        plot_frame.grid(row=0, column=1, sticky="nsew")
        self.grid_columnconfigure(1, weight=1)

        self.kp_var = tk.DoubleVar(value=25.0)
        self.ki_var = tk.DoubleVar(value=18.0)
        self.kd_var = tk.DoubleVar(value=2.5)
        self.setpoint_var = tk.DoubleVar(value=90.0)

        ttk.Label(controls_frame, text="Proportional (Kp)").pack()
        ttk.Scale(controls_frame, from_=0, to=50, variable=self.kp_var, orient='horizontal').pack(fill='x', expand=True)
        ttk.Label(controls_frame, text="Integral (Ki)").pack()
        ttk.Scale(controls_frame, from_=0, to=40, variable=self.ki_var, orient='horizontal').pack(fill='x', expand=True)
        ttk.Label(controls_frame, text="Derivative (Kd)").pack()
        ttk.Scale(controls_frame, from_=0, to=10, variable=self.kd_var, orient='horizontal').pack(fill='x', expand=True)
        ttk.Label(controls_frame, text="Setpoint (degrees)").pack()
        ttk.Scale(controls_frame, from_=0, to=180, variable=self.setpoint_var, orient='horizontal').pack(fill='x', expand=True)
        ttk.Button(controls_frame, text="Reset Simulation", command=self.reset_simulation).pack(pady=20)
        
        self.fig = Figure(figsize=(10, 8), dpi=100)
        self.ax_arm = self.fig.add_subplot(2, 1, 1)
        self.ax_plot = self.fig.add_subplot(2, 1, 2)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

        self.reset_simulation()
        # OPTIMIZATION: Use blit=True for performance. init_func sets up the static background.
        self.anim = animation.FuncAnimation(self.fig, self.update_plot, init_func=self._init_plot,
                                            interval=self.dt * 1000, blit=True, cache_frame_data=False)

    def _init_plot(self):
        """Initializes the static parts of the plot for blitting."""
        # Setup arm plot
        self.ax_arm.set_xlim(-1.2, 1.2); self.ax_arm.set_ylim(-0.2, 1.2)
        self.ax_arm.set_aspect('equal'); self.ax_arm.grid(True)
        self.arm_line, = self.ax_arm.plot([], [], 'b-', lw=5, label='Arm')
        self.setpoint_line, = self.ax_arm.plot([], [], 'r--', lw=2, label='Setpoint')
        self.arm_title = self.ax_arm.set_title("Arm Angle: 0.0°")
        self.ax_arm.legend(loc='upper right')

        # Setup data plot
        self.ax2 = self.ax_plot.twinx()
        self.ax_plot.set_xlabel("Time (s)"); self.ax_plot.set_ylabel("Angle (degrees)", color='b')
        self.ax2.set_ylabel("Motor Output (%)", color='g'); self.ax_plot.grid(True)
        self.ax_plot.set_ylim(-10, 190); self.ax2.set_ylim(-110, 110)
        
        self.angle_line, = self.ax_plot.plot([], [], 'b-', label='Angle (°)')
        self.output_line, = self.ax2.plot([], [], 'g:', alpha=0.5, label='Motor Output (%)')
        
        # Return a list of all artists that will be updated in the animation
        return [self.arm_line, self.setpoint_line, self.angle_line, self.output_line, self.arm_title]

    def reset_simulation(self):
        """Resets the arm, PID controller, and data logs."""
        self.arm = RobotArm(initial_angle=0.0)
        self.pid = PID(self.kp_var.get(), self.ki_var.get(), self.kd_var.get(), self.setpoint_var.get())
        self.times.clear(); self.angles.clear(); self.setpoints.clear(); self.outputs.clear()
        self.last_time = 0

    def update_plot(self, frame):
        """This function is called by the animation loop. It only updates artist data."""
        # Update PID controller from sliders
        self.pid.Kp = self.kp_var.get(); self.pid.Ki = self.ki_var.get(); self.pid.Kd = self.kd_var.get()
        self.pid.setpoint = self.setpoint_var.get()
        
        # Run simulation step
        motor_output = self.pid.update(self.arm.angle, self.dt)
        self.arm.update(motor_output, self.dt)
        
        # Update data
        self.last_time += self.dt
        self.times.append(self.last_time); self.angles.append(self.arm.angle)
        self.setpoints.append(self.pid.setpoint); self.outputs.append(motor_output)

        # --- Update Artists (this is fast) ---
        # Update arm visualization
        arm_angle_rad, setpoint_angle_rad = np.radians(self.arm.angle), np.radians(self.pid.setpoint)
        self.arm_line.set_data([0, np.cos(arm_angle_rad)], [0, np.sin(arm_angle_rad)])
        self.setpoint_line.set_data([0, np.cos(setpoint_angle_rad)], [0, np.sin(setpoint_angle_rad)])
        self.arm_title.set_text(f"Arm Angle: {self.arm.angle:.1f}°")

        # Update data plots
        self.angle_line.set_data(self.times, self.angles)
        self.output_line.set_data(self.times, self.outputs)
        
        # Dynamically update x-axis limits
        if self.last_time > self.max_data_points * self.dt:
            xmin = self.last_time - self.max_data_points * self.dt
            self.ax_plot.set_xlim(xmin, self.last_time)
        else:
            self.ax_plot.set_xlim(0, self.max_data_points * self.dt)

        return [self.arm_line, self.setpoint_line, self.angle_line, self.output_line, self.arm_title]

if __name__ == "__main__":
    app = PIDSimulatorApp()
    app.mainloop()