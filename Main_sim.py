import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from vpython import *
from time import *
import math
import pandas as pd
from matplotlib.gridspec import GridSpec

def create_orientation_program(dt = 0.004, total_time = 2, duration_of_commands = 0.5, transition_time = 0.1, number_of_smoothing = 1,  *command_tuple):
    total_number_of_data = int(total_time/dt)    
    number_of_commands = len(command_tuple)
    
    rolls = np.zeros(total_number_of_data)
    pitchs = np.zeros(total_number_of_data)


    temps =  [dt*i for i in range(0, total_number_of_data)]


    for index in range(number_of_commands):
        for i, t in enumerate(temps):
            if t < duration_of_commands*index:
                continue
            if (t < duration_of_commands*(index+1)):
                rolls[i] = command_tuple[index][0]
                pitchs[i] = command_tuple[index][1]
            else:
                break

    if transition_time > 0:
        number_of_data_for_transition = int(transition_time/dt)

        for i in range(number_of_smoothing):
            rolls = pd.Series(rolls).rolling(window=number_of_data_for_transition, min_periods=1, center = True).mean()
            pitchs = pd.Series(pitchs).rolling(window=number_of_data_for_transition, min_periods=1, center = True).mean()


    return temps, rolls, pitchs


class Imu:
    def __init__(self):
        self.roll_offset = 0
        self.pitch_offset = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def noise_function(self, time):
        return np.sin(4 * time) + 0.3 * np.sin(40 * time)

    def compute_imu_angles_and_noise(self, roll, pitch, yaw, time):

        noise_fonc_cap = self.noise_function(time)

        imu_noise = False

        if imu_noise:
            roll_noise = np.random.random() * noise_fonc_cap
            pitch_noise = np.random.random() * noise_fonc_cap
            yaw_noise = np.random.random() * noise_fonc_cap
        else:
            roll_noise = 0
            pitch_noise = 0
            yaw_noise = 0

        self.roll = roll + self.roll_offset + roll_noise
        self.pitch = pitch + self.pitch_offset + pitch_noise
        self.yaw = yaw + yaw_noise

        return roll, pitch, yaw


class motor:
    def __init__(self, x, y, thrust_fraction):
        g = 9.81
        motor_nominal_thrust = 0.8  # kg

        self.pos_to_center = np.array([x, y, 0])
        self.thrust_fraction = thrust_fraction
        self.max_thrust = (
            g * motor_nominal_thrust * self.thrust_fraction
        )  # To newtons and scaled with thrust fraction

        self.pwm = 1512


class simulation:
    def __init__(self, dt, sim_time, mode):
        self.dt = dt
        self.time = 0
        self.sim_time = sim_time
        self.mode = mode

        self.imu = Imu()
        self.drone_software = Drone_software()
        self.drone_software.reset_pid()

        # real parameters
        Ixx = 3.14e-3
        Iyy = Ixx
        Izz = 2.94e-3
        self.inverse_inertial_matrix = np.array(
            [[1 / Ixx, 0, 0], [0, 1 / Iyy, 0], [0, 0, 1 / Izz]]
        )

        self.mass = 0.840
        self.center_of_mass = np.array([0.0, 0.0, 0.0])
        self.i = 0

        self.first = True

        # fraction of power motor has
        # ADJUST POSITIONS
        self.motor1 = motor(0.10, 0.10, 1.0)
        self.motor2 = motor(-0.10, 0.10, 1.0)
        self.motor3 = motor(-0.10, -0.10, 1.0)
        self.motor4 = motor(0.10, -0.10, 1.0)

        self.motors = [self.motor1, self.motor2, self.motor3, self.motor4]

        self.initialize()
        self.set_up_scene()

        np.random.seed(42)

        self.run_mainloop()

    def run_mainloop(self):
        temp, roll, pitch = create_orientation_program(self.dt, self.sim_time, 0.3, 0.1, 2, (0,0), (10,0), (-10,0), (0,0), (0,25), (25,0),(30,0))

        while self.time < self.sim_time:
            RateRoll = self.angular_speed[0]
            RatePitch = self.angular_speed[1]
            RateYaw = self.angular_speed[2]

            try:
                self.DesiredRoll, self.DesiredPitch = (  
                    roll[self.i],
                    pitch[self.i],
                )
            except:
                self.DesiredRoll, self.DesiredPitch = (  
                    roll[self.i-1],
                    pitch[self.i-1],
                )

            self.drone_software.apply_PID_loops(
                self.DesiredRoll,
                self.DesiredPitch,
                self.imu.roll,
                self.imu.pitch,
                RateRoll,
                RatePitch,
                RateYaw,
            )
            self.motor1.pwm, self.motor2.pwm, self.motor3.pwm, self.motor4.pwm = (
                self.drone_software.compute_motor_inputs(self.throttle)
            )
            self.roll_pid = self.drone_software.InputRoll
            self.pitch_pid = self.drone_software.InputPitch

            self.compute_motor_forces()
            self.compute_force_and_torque()
            self.compute_dynamics()
            self.record_data()
            self.visualize(self.q, self.drone_speed * self.dt, self.drone_pos)

            self.time += self.dt
            self.i += 1

    def set_up_scene(self):
        scene.range = 5
        scene.background = color.yellow
        toRad =  np.pi / 180
        toDeg = 1 / toRad
        scene.forward = vector(-1, -1, -1)

        scene.width = 1200
        scene.height = 1080

        self.xarrow = arrow(
            lenght=2, shaftwidth=0.1, color=color.red, axis=vector(1, 0, 0)
        )
        self.yarrow = arrow(
            lenght=2, shaftwidth=0.1, color=color.green, axis=vector(0, 1, 0)
        )
        self.zarrow = arrow(
            lenght=4, shaftwidth=0.1, color=color.blue, axis=vector(0, 0, 1)
        )

        self.frontArrow = arrow(
            length=4, shaftwidth=0.05, color=color.purple, axis=vector(1, 0, 0)
        )
        self.upArrow = arrow(
            length=1, shaftwidth=0.05, color=color.magenta, axis=vector(0, 1, 0)
        )
        self.sideArrow = arrow(
            length=2, shaftwidth=0.05, color=color.orange, axis=vector(0, 0, 1)
        )
        self.speedArrow = arrow(
            length=0, shaftwidth=0.08, color=color.black, axis=vector(0, 0, 1)
        )
        self.accelArrow = arrow(
            length=0, shaftwidth=0.08, color=color.cyan, axis=vector(0, 0, 1)
        )

        bBoard = box(
            length=6,
            width=2,
            height=0.2,
            opacity=0.8,
            pos=vector(
                0,
                0,
                0,
            ),
        )
        bn = box(
            length=1,
            width=0.75,
            height=0.1,
            pos=vector(-0.5, 0.1 + 0.05, 0),
            color=color.blue,
        )
        nano = box(
            lenght=1.75,
            width=0.6,
            height=0.1,
            pos=vector(-2, 0.1 + 0.05, 0),
            color=color.green,
        )
        self.myObj = compound([bBoard, bn, nano])

    def compute_motor_forces(self):
        for motor in self.motors:
            motor.thrust = motor.max_thrust * ((motor.pwm - 1000) / 1000) ** 2
        # Assumed quadratic relation between pwm and thrust

    def compute_force_and_torque(self):
        if self.i == 0:
            self.normal = np.array([0, 0, 1])

        self.force = (
            self.motor1.thrust
            + self.motor2.thrust
            + self.motor3.thrust
            + self.motor4.thrust
        )
        self.force_array = self.normal * self.force

        air_density = 1.225 #Kg/m^3
        C_d = 1.1
        Area_cross_section = 0.15*0.15

        Drag_force = -0.5*air_density*C_d*Area_cross_section*self.drone_speed*np.linalg.norm(self.drone_speed)

        self.force_array += Drag_force
        

        self.torque = np.array([0.0, 0.0, 0.0])
        for motor in self.motors:
            self.torque = self.torque + np.cross(
                (motor.pos_to_center - self.center_of_mass), [0, 0, motor.thrust]
            )

    def compute_dynamics(self):

        gravity_force = np.array([0, 0, -self.mass * 9.81])
        self.drone_acc = (self.force_array + gravity_force) / self.mass
        self.drone_speed = self.drone_speed + self.drone_acc * self.dt
        self.drone_pos = self.drone_pos + self.drone_speed * self.dt

        self.angular_accel = self.inverse_inertial_matrix @ self.torque
        self.angular_speed = self.angular_speed + self.angular_accel * self.dt

        
        if self.i == 0:
            self.q = np.array([1, 0, 0, 0])

        dq, self.q = update_orientation(self.q, self.angular_speed, self.dt)
        self.normal = normalize_quaternion(rotate_vector(self.normal, dq))

        roll, pitch, yaw = quaternion_to_euler(self.q)
        roll *= 180 / np.pi
        pitch *= 180 / np.pi
        yaw *= 180/np.pi

        self.imu.compute_imu_angles_and_noise(roll, pitch, yaw, self.time)

    def record_data(self):
        if self.first:
            self.data = {
                "time": [],
                "pos": [],
                "speed": [],
                "acc": [],
                "angular_acc": [],
                "angular_speed": [],
                "normal": [],
                "roll": [],
                "pitch": [],
                "command_roll": [],
                "command_pitch": [],
                "roll_pid": [],
                "pitch_pid": [],
            }
            self.first = False
        else:
            self.data["time"].append(self.time)

            self.data["pos"].append(self.drone_pos)
            self.data["speed"].append(self.drone_speed)
            self.data["acc"].append(self.drone_acc)

            self.data["normal"].append(self.normal)
            self.data["angular_speed"].append(self.angular_speed)
            self.data["angular_acc"].append(self.drone_acc)

            self.data["roll"].append(self.imu.roll)
            self.data["pitch"].append(self.imu.pitch)

            self.data["command_roll"].append(self.DesiredRoll)
            self.data["command_pitch"].append(self.DesiredPitch)

            self.data["roll_pid"].append(self.roll_pid)
            self.data["pitch_pid"].append(self.pitch_pid)

            # print(f' Time : {self.time} , Normal : {self.normal}')

    def initialize(self):

        self.drone_pos = np.array([0.0, 0.0, 0.0])
        self.drone_speed = np.array([0.0, 0.0, 0.0])
        self.drone_acc = np.array([0.0, 0.0, 0.0])
        self.angular_accel = np.array([0, 0, 0])
        self.angular_speed = np.array([0, 0, 0])

        self.throttle = 1500

    def visualize(self, quaternion, speed_times_dt, pos):
        try:
            q0 = float(quaternion[0])
            q1 = float(quaternion[1])
            q2 = float(quaternion[2])
            q3 = float(quaternion[3])

            roll = -math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
            pitch = math.asin(2 * (q0 * q2 - q3 * q1))
            yaw = (
                -math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
                - np.pi / 2
            )

            rate(50)
            k = vector(cos(yaw) * cos(pitch), sin(pitch), sin(yaw) * cos(pitch))
            y = vector(0, 1, 0)
            s = cross(k, y)
            v = cross(s, k)
            vrot = v * cos(roll) + cross(k, v) * sin(roll)

            self.frontArrow.axis = k
            self.sideArrow.axis = cross(k, vrot)
            self.upArrow.axis = vrot
            self.myObj.axis = k
            self.myObj.up = vrot
            self.sideArrow.length = 2
            self.frontArrow.length = 4
            self.upArrow.length = 1

            self.speedArrow.length = np.linalg.norm(self.drone_speed)
            self.speedArrow.axis = vector(self.drone_speed[1], self.drone_speed[2], self.drone_speed[0])

            self.accelArrow.length = np.linalg.norm(self.drone_acc)
            self.accelArrow.axis = vector(self.drone_acc[1], self.drone_acc[2], self.drone_acc[0])


            # self.drone_position += vector(speed_times_dt[0], speed_times_dt[1], speed_times_dt[2])
            # myObj.pos = vector(self.drone)  # Move the object to the new position
        except:
            pass


class Drone_software:
    def __init__(self):
        # rate pid values
        self.PRateRoll, self.PRatePitch, self.PRateYaw = 5, 5, 0.0
        self.IRateRoll, self.IRatePitch, self.IRateYaw = 1, 1, 0.0
        self.DRateRoll, self.DRatePitch, self.DRateYaw = 0.11, 0.11, 0.0

        # angle pid values
        self.PRoll, self.PPitch = 2.5, 2.5
        self.IRoll, self.IPitch = 0.0, 0.0
        self.DRoll, self.DPitch = 0.1, 0.1

    def pid_equation(self, Error, P, I, D, PrevError, PrevIterm):
        MaxPIDvalues = 300
        time_passed = 0.004

        Pterm = P * Error
        Iterm = PrevIterm + I * (Error + PrevError) * time_passed / 2
        Dterm = D * (Error - PrevError) / time_passed

        integralMax = MaxPIDvalues / 2
        if Iterm > integralMax:
            Iterm = integralMax
        elif Iterm < -integralMax:
            Iterm = -integralMax

        PIDOutput = Pterm + Iterm + Dterm

        if PIDOutput > MaxPIDvalues:
            PIDOutput = MaxPIDvalues

            # un peu louche comme ligne...
            Iterm = PrevIterm
        elif PIDOutput < -MaxPIDvalues:
            PIDOutput = -MaxPIDvalues
            # loouche
            Iterm = -PrevIterm

        return PIDOutput, Error, Iterm

    def apply_PID_loops(
        self,
        DesiredRoll,
        DesiredPitch,
        madwickroll,
        madwickpitch,
        RateRoll,
        RatePitch,
        RateYaw,
    ):

        ErrorRoll = DesiredRoll - madwickroll
        ErrorPitch = DesiredPitch - madwickpitch

        DesiredRateRoll, self.PrevErrorRoll, self.PrevItermRoll = self.pid_equation(
            ErrorRoll,
            self.PRoll,
            self.IRoll,
            self.DRoll,
            self.PrevErrorRoll,
            self.PrevItermRoll,
        )
        DesiredRatePitch, self.PrevErrorPitch, self.PrevItermPitch = self.pid_equation(
            ErrorPitch,
            self.PPitch,
            self.IPitch,
            self.DPitch,
            self.PrevErrorPitch,
            self.PrevItermPitch,
        )

        # ChangeRateYaw
        DesiredRateYaw = 0

        ErrorRateRoll = DesiredRateRoll - RateRoll
        ErrorRatePitch = DesiredRatePitch - RatePitch
        ErrorRateYaw = DesiredRateYaw - RateYaw

        self.InputRoll, self.PrevErrorRateRoll, self.PrevItermRateRoll = (
            self.pid_equation(
                ErrorRateRoll,
                self.PRateRoll,
                self.IRateRoll,
                self.DRateRoll,
                self.PrevErrorRateRoll,
                self.PrevItermRateRoll,
            )
        )
        self.InputPitch, self.PrevErrorRatePitch, self.PrevItermRatePitch = (
            self.pid_equation(
                ErrorRatePitch,
                self.PRatePitch,
                self.IRatePitch,
                self.DRatePitch,
                self.PrevErrorRatePitch,
                self.PrevItermRatePitch,
            )
        )
        self.InputYaw, self.PrevErrorRateYaw, self.PrevItermRateYaw = self.pid_equation(
            ErrorRateYaw,
            self.PRateYaw,
            self.IRateYaw,
            self.DRateYaw,
            self.PrevErrorRateYaw,
            self.PrevItermRateYaw,
        )

    def reset_pid(self):
        self.PrevErrorRateRoll = 0.0
        self.PrevErrorRatePitch = 0.0
        self.PrevErrorRateYaw = 0.0

        self.PrevItermRateRoll = 0.0
        self.PrevItermRatePitch = 0.0
        self.PrevItermRateYaw = 0.0

        self.PrevErrorRoll = 0.0
        self.PrevErrorPitch = 0.0

        self.PrevItermRoll = 0.0
        self.PrevItermPitch = 0.0

    def compute_motor_inputs(self, throttle):
        # 1. Control mixer
        multiplication_factor = 1.024  # Check Carbon Aeronautics for more info
        MotorInput1 = multiplication_factor * (
            throttle - self.InputRoll - self.InputPitch - self.InputYaw
        )
        MotorInput2 = multiplication_factor * (
            throttle + self.InputRoll - self.InputPitch + self.InputYaw
        )
        MotorInput3 = multiplication_factor * (
            throttle + self.InputRoll + self.InputPitch - self.InputYaw
        )
        MotorInput4 = multiplication_factor * (
            throttle - self.InputRoll + self.InputPitch + self.InputYaw
        )

        # 2. Limit max and min thrust
        if MotorInput1 > 2000:
            MotorInput1 = 1999
        if MotorInput2 > 2000:
            MotorInput2 = 1999
        if MotorInput3 > 2000:
            MotorInput3 = 1999
        if MotorInput4 > 2000:
            MotorInput4 = 1999

        ThrottleIdle = 1200  # Minimum spinning throttle
        if MotorInput1 < ThrottleIdle:
            MotorInput1 = ThrottleIdle
        if MotorInput2 < ThrottleIdle:
            MotorInput2 = ThrottleIdle
        if MotorInput3 < ThrottleIdle:
            MotorInput3 = ThrottleIdle
        if MotorInput4 < ThrottleIdle:
            MotorInput4 = ThrottleIdle

        return MotorInput1, MotorInput2, MotorInput3, MotorInput4


def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    if norm == 0:
        return q
    return q / norm


def update_orientation(q, omega, dt):
    omega = rotate_vector(omega, q)
    theta = np.linalg.norm(omega)
    if theta == 0:
        return np.array([1, 0, 0, 0]), np.array([1, 0, 0, 0])
    else:
        sin_half_theta = np.sin(theta * dt / 2)
        dq = np.array(
            [
                np.cos(theta * dt / 2),
                sin_half_theta * omega[0] / theta,
                sin_half_theta * omega[1] / theta,
                sin_half_theta * omega[2] / theta,
            ]
        )
        dq = normalize_quaternion(dq)
        q = quaternion_multiply(dq, q)
        q = normalize_quaternion(q)
        return dq, q


def rotate_vector(v, q):
    v_quat = np.array([0] + list(v))  # Create a quaternion with zero scalar part
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])  # Conjugate of q
    v_rotated_quat = quaternion_multiply(quaternion_multiply(q, v_quat), (q_conj))
    return v_rotated_quat[1:]  # Extract the vector part


def quaternion_to_euler(q):
    w, x, y, z = q

    q0, q1, q2, q3 = w, x, y, z

    # Roll (φ)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))

    # Pitch (θ)
    pitch = np.arcsin(2 * (w * y - z * x))

    yaw = -math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) - np.pi / 2

    return roll, pitch, yaw


sim = simulation(0.004, 4, "some_mode")


# Prepare data
trajectory = np.array(sim.data["pos"])
x = trajectory[:, 0]
y = trajectory[:, 1]
z = trajectory[:, 2]

# Create a new figure with 6 subplots (2x3 grid)
fig = plt.figure(figsize=(15, 12))

gs = GridSpec(2, 3, width_ratios=[1, 1, 1], height_ratios=[1, 1], hspace=0.3, wspace=0.2)

# First subplot: 3D trajectory
ax1 = fig.add_subplot(gs[0, 0], projection="3d")
scatter_size = 2 
ax1.scatter(x, y, z, label="Trajectory", s=scatter_size, c="b", marker="o")
ax1.set_xlabel("X Label")
ax1.set_ylabel("Y Label")
ax1.set_zlabel("Z Label")
ax1.set_xlim([-4, 4])
ax1.set_ylim([-4, 4])
ax1.set_zlim([0, 8])
ax1.legend()

# Second subplot: Roll vs command_roll
ax2 = fig.add_subplot(gs[0, 1])
ax2.plot(sim.data["time"], sim.data["roll"], label="actual roll", color="b")
ax2.plot(sim.data["time"], sim.data["command_roll"], label="command", color="r", linestyle="--")
ax2.set_title("Roll vs Command Roll")
ax2.set_xlabel("Time")
ax2.set_ylabel("Roll")
ax2.legend()

# Third subplot: Pitch vs command_pitch
ax3 = fig.add_subplot(gs[0, 2])
ax3.plot(sim.data["time"], sim.data["pitch"], label="actual pitch", color="b")
ax3.plot(sim.data["time"], sim.data["command_pitch"], label="command", color="r", linestyle="--")
ax3.set_title("Pitch vs Command Pitch")
ax3.set_xlabel("Time")
ax3.set_ylabel("Pitch")
ax3.legend()

# Fourth subplot: Roll PID value
ax4 = fig.add_subplot(gs[1, 1])
ax4.plot(sim.data["time"], sim.data["roll_pid"], color="b")
ax4.set_title("Roll PID Value")
ax4.set_xlabel("Time")
ax4.set_ylabel("Roll PID")

# Fifth subplot: Pitch PID value
ax5 = fig.add_subplot(gs[1, 2])
ax5.plot(sim.data["time"], sim.data["pitch_pid"], color="b")
ax5.set_title("Pitch PID Value")
ax5.set_xlabel("Time")
ax5.set_ylabel("Pitch PID")


plt.subplots_adjust(left=0, right=0.95, top=0.95, bottom=0.05, wspace=0.4, hspace=0.4)
# Show the combined figure
plt.show()
