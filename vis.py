import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class Imu():
    def __init__(self):
        self.roll_offset = 0
        self.pitch_offset = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        

    def compute_noise(self, roll, pitch, yaw):
        roll_noise = 0
        pitch_noise = 0
        yaw_noise = 0

        self.roll = roll + self.roll_offset + roll_noise
        self.pitch = pitch + self.pitch_offset + pitch_noise
        self.yaw = yaw + yaw_noise
        print('roulis : ')
        print(self.roll)

        return roll, pitch, yaw

class motor:
    def __init__(self,x,y,thrust_fraction):
        g = 9.81
        motor_nominal_thrust = 0.8 #kg

        self.x = x
        self.y = y

        self.pos_to_center= np.array([x,y,0])
        self.thrust_fraction = thrust_fraction
        self.max_thrust = g*motor_nominal_thrust*self.thrust_fraction #To newtons and scaled with thrust fraction

        self.pwm = 1550
        self.thrust = 0

class simulation:
    def __init__(self, dt, time, mode):
        self.dt = dt
        self.time = 0
        self.timer = time
        self.mode = mode

        self.imu = Imu()
        self.drone_software = Drone_software()
        self.drone_software.reset_pid()

        #real parameters
        self.drone_pos = np.array([0.,0.,0.])
        self.drone_speed = np.array([0.,0.,0.])
        self.drone_acc = np.array([0.,0.,0.])
        Ixx = 3.14e-3
        Iyy = Ixx
        Izz = 2.94e-3
        self.inverse_inertial_matrix = np.array([[1/Ixx, 0, 0],
                                                 [0, 1/Iyy, 0],
                                                 [0, 0, 1/Izz]])

        self.mass = 0.840
        self.center_of_mass = np.array([0.,0.,0.])
        self.i = 0
        

        self.first = True

        #fraction of power motor has
        #ADJUST POSITIONS
        self.motor1 = motor(0.10,0.10,1.05)
        self.motor2 = motor(0.10,-0.10,1.0)
        self.motor3 = motor(-0.10,-0.10,1.0)
        self.motor4 = motor(-0.10,0.10,1.0)

        self.motors = [self.motor1, self.motor2, self.motor3, self.motor4]


        self.initialize()

        self.run_mainloop()

    def run_mainloop(self):
        DesiredRoll = 0
        DesiredPitch = 0

        while (self.time < self.timer):
            RateRoll = self.angular_speed[0]
            RatePitch = self.angular_speed[1]
            RateYaw = self.angular_speed[2]

            self.drone_software.apply_PID_loops(DesiredRoll, DesiredPitch, self.imu.roll, self.imu.pitch, RateRoll, RatePitch, RateYaw)
            self.motor1.pwm, self.motor2.pwm, self.motor3.pwm, self. motor4.pwm = self.drone_software.compute_motor_inputs(self.throttle)
        
            self.compute_motor_forces()
            self.compute_force_and_torque()
            self.compute_dynamics()
            self.record_data()

            self.time += self.dt
            self.i +=1

    def compute_motor_forces(self):
        for motor in self.motors:
            motor.thrust = motor.max_thrust*((motor.pwm - 1000)/1000)**2 
        #Assumed quadratic relation between pwm and thrust

    def compute_force_and_torque(self):
        if (self.i == 0):
            self.normal = np.array([0,0,1])

        self.force = self.motor1.thrust + self.motor2.thrust + self.motor3.thrust + self.motor4.thrust
        self.force_array = self.normal*self.force

        self.torque = np.array([0.,0.,0.])
        for motor in self.motors:
            self.torque = self.torque + np.cross((motor.pos_to_center-self.center_of_mass), [0,0,motor.thrust])

    def compute_dynamics(self):
        
        gravity_force = np.array([0,0,-self.mass*9.81])
        self.drone_acc = (self.force_array + gravity_force)/self.mass
        self.drone_speed = self.drone_speed + self.drone_acc*self.dt 
        self.drone_pos = self.drone_pos + self.drone_speed*self.dt

        #need to implement quaternion multiplication
        self.angular_accel = self.inverse_inertial_matrix @ self.torque
        self.angular_speed = self.angular_speed + self.angular_accel*self.dt
        if self.i == 0:
            self.q = np.array([1, 0, 0, 0])

        dq, self.q = update_orientation(self.q, self.angular_speed, self.dt)
        self.normal = normalize_quaternion(rotate_vector(self.normal, dq))

       

        roll, pitch = quaternion_to_euler(self.q)
        roll *= 180/np.pi
        pitch *= 180/np.pi

        yaw = 0
        self.imu.compute_noise(roll, pitch, yaw)
        


    def record_data(self):
        if self.first:
            self.data = {
                'pos' : [],
                'speed' : [],
                'acc' : [],
                
                'angular_acc' : [],
                'angular_speed' : [],
                'normal' : [],

                'roll' : [],
                'pitch' : [],
            }
            self.first = False
        else:
            self.data['pos'].append(self.drone_pos)
            self.data['speed'].append(self.drone_speed)
            self.data['acc'].append( self.drone_acc)

            self.data['normal'].append(self.normal)
            self.data['angular_speed'].append(self.angular_speed)
            self.data['angular_acc'].append( self.drone_acc)

            self.data['roll'].append(self.imu.roll)
            self.data['pitch'].append(self.imu.pitch)
            print(f' Time : {self.time} , Normal : {self.normal}')


    def initialize(self):
        self.x = 0
        self.y = 0
        self.z = 0

        self.vx = 0
        self.vy = 0
        self.vz = 0
        
        self.ax = 0
        self.ay = 0
        self.az = 0

        self.F1 = 0
        self.F2 = 0
        self.F3 = 0
        self.F3 = 0
        

        self.angular_accel = np.array([0,0,0])
        self.angular_speed = np.array([0,0,0])

        self.throttle = 1500



class Drone_software():
    def __init__(self):
        #rate pid values
        self.PRateRoll, self.PRatePitch, self.PRateYaw = 1., 1., 0.
        self.IRateRoll, self.IRatePitch, self.IRateYaw = 0., 0., 0.
        self.DRateRoll, self.DRatePitch,self.DRateYaw = 0., 0., 0.

        #angle pid values
        self.PRoll, self.PPitch = 2., 2.
        self.IRoll, self.IPitch = 0., 0.
        self.DRoll, self.DPitch = 0., 0.


    def pid_equation(self,Error, P, I, D, PrevError, PrevIterm):
        MaxPIDvalues = 300
        time_passed = 0.004

        Pterm = P*Error
        Iterm = PrevIterm + I*(Error + PrevError)*time_passed/2
        Dterm = D*(Error - PrevError)/time_passed

        integralMax = MaxPIDvalues/2
        if (Iterm > integralMax):
            Iterm = integralMax
        elif (Iterm < -integralMax):
            Iterm = -integralMax

        PIDOutput = Pterm + Iterm + Dterm

        if (PIDOutput > MaxPIDvalues):
            PIDOutput = MaxPIDvalues

            #un peu louche comme ligne...
            Iterm = PrevIterm
        elif (PIDOutput < -MaxPIDvalues):
            PIDOutput = -MaxPIDvalues
            #loouche
            Iterm = -PrevIterm

        return PIDOutput, Error, Iterm

    def apply_PID_loops(self, DesiredRoll, DesiredPitch, madwickroll, madwickpitch, RateRoll, RatePitch, RateYaw):

        ErrorRoll = DesiredRoll - madwickroll
        ErrorPitch = DesiredPitch - madwickpitch 

        DesiredRateRoll, self.PrevErrorRoll, self.PrevItermRoll = self.pid_equation(ErrorRoll, self.PRoll, self.IRoll, self.DRoll, self.PrevErrorRoll, self.PrevItermRoll)
        DesiredRatePitch, self.PrevErrorPitch, self.PrevItermPitch = self.pid_equation(ErrorPitch, self.PPitch, self.IPitch, self.DPitch, self.PrevErrorPitch, self.PrevItermPitch)

        #ChangeRateYaw
        DesiredRateYaw = 0

        ErrorRateRoll = DesiredRateRoll - RateRoll
        ErrorRatePitch = DesiredRatePitch - RatePitch
        ErrorRateYaw = DesiredRateYaw - RateYaw

        self.InputRoll, self.PrevErrorRateRoll, self.PrevItermRateRoll = self.pid_equation(ErrorRateRoll, self.PRateRoll, self.IRateRoll, self.DRateRoll, self.PrevErrorRateRoll, self.PrevItermRateRoll)
        self.InputPitch, self.PrevErrorRatePitch, self.PrevItermRatePitch = self.pid_equation(ErrorRatePitch, self.PRatePitch, self.IRatePitch, self.DRatePitch, self.PrevErrorRatePitch, self.PrevItermRatePitch)
        self.InputYaw, self.PrevErrorRateYaw, self.PrevItermRateYaw = self.pid_equation(ErrorRateYaw, self.PRateYaw, self.IRateYaw, self.DRateYaw, self.PrevErrorRateYaw, self.PrevItermRateYaw)

    def reset_pid(self):
        self.PrevErrorRateRoll = 0.
        self.PrevErrorRatePitch = 0.
        self.PrevErrorRateYaw = 0.

        self.PrevItermRateRoll = 0.
        self.PrevItermRatePitch = 0.
        self.PrevItermRateYaw = 0.


        self.PrevErrorRoll = 0.
        self.PrevErrorPitch = 0.
        
        self.PrevItermRoll = 0.
        self.PrevItermPitch = 0.
    
    def compute_motor_inputs(self, throttle):
        # 1. Control mixer
        multiplication_factor = 1.024  # Check Carbon Aeronautics for more info
        MotorInput1 = multiplication_factor * (throttle - self.InputRoll - self.InputPitch - self.InputYaw)
        MotorInput2 = multiplication_factor * (throttle + self.InputRoll - self.InputPitch + self.InputYaw)
        MotorInput3 = multiplication_factor * (throttle + self.InputRoll + self.InputPitch - self.InputYaw)
        MotorInput4 = multiplication_factor * (throttle - self.InputRoll + self.InputPitch + self.InputYaw)

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
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])
        

def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    if norm == 0:
        return q
    return q / norm


def update_orientation(q, omega, dt):
    omega = rotate_vector(omega,q)
    theta = np.linalg.norm(omega)
    if theta == 0:
        return np.array([1,0,0,0]), np.array([1,0,0,0])
    else:
        sin_half_theta = np.sin(theta*dt/2)
        dq = np.array([np.cos(theta*dt/2), sin_half_theta*omega[0]/theta, sin_half_theta*omega[1]/theta, sin_half_theta*omega[2]/theta])
        dq = normalize_quaternion(dq)
        q = quaternion_multiply(dq,q)
        q = normalize_quaternion(q)
        return dq, q

        
def rotate_vector(v, q):
    v_quat = np.array([0] + list(v))  # Create a quaternion with zero scalar part
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])  # Conjugate of q
    v_rotated_quat = quaternion_multiply(quaternion_multiply(q, v_quat), (q_conj))
    return v_rotated_quat[1:]  # Extract the vector part



def quaternion_to_euler(q):
    w, x, y, z = q
    
    # Roll (φ)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    
    # Pitch (θ)
    pitch = np.arcsin(2 * (w * y - z * x))
    
    return roll, pitch


sim = simulation(0.004, 2, 'some_mode')




trajectory = np.array(sim.data['pos'])

x = trajectory[:, 0]
y = trajectory[:, 1]
z = trajectory[:, 2]

# Create a new figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the trajectory with scatter points and control the size with 's'
scatter_size = 8  # Adjust this value to make points smaller or larger
ax.scatter(x, y, z, label='Trajectory', s=scatter_size, c='b', marker='o')

# Add labels for the axes
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
ax.set_xlim([0, 10])
ax.set_ylim([0, 10])
ax.set_zlim([0, 10])

# Add a legend
ax.legend()

# Show the interactive plot
plt.show()