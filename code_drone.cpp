/*
Code initial écrit par Colin Rousseau, dans le cadre du projet Icarus, grandement inspiré de la logique de contrôle amenée par Carbon Aeronautics
*/
//next line for setting up more than 2 interrupts on arduino nano
#include <EnableInterrupt.h>
#include <Wire.h>
//#include <PulsePosition.h>
#include <Fusion.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

float AccX, AccY, AccZ = 0.;
float GyroX, GyroY, GyroZ = 0.;
Adafruit_MPU6050 mpu;

// Add low-pass filter coefficients (tunable)
const float accelFilterAlpha = 1.; // Low-pass filter coefficient for accelerometer
const float gyroFilterAlpha = 1.; // Low-pass filter coefficient for gyroscope
// Filtered sensor values
float filteredAccX, filteredAccY, filteredAccZ;
float filteredGyroX, filteredGyroY, filteredGyroZ;


#define RCPin1 2
#define RCPin2 3
#define RCPin3 4
#define RCPin4 5

Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

volatile long startTime1 = 0;
volatile long currentTime1 = 0;
volatile long pulses1 = 0;
int pulseWidth1 = 0;

volatile long startTime2 = 0;
volatile long currentTime2 = 0;
volatile long pulses2 = 0;
int pulseWidth2 = 0;


volatile long startTime3 = 0;
volatile long currentTime3 = 0;
volatile long pulses3 = 0;
int pulseWidth3 = 0;

volatile long startTime4 = 0;
volatile long currentTime4 = 0;
volatile long pulses4 = 0;
int pulseWidth4 = 0;


FusionAhrs ahrs; // initialize fusion algo
// Set AHRS algorithm settings

const FusionAhrsSettings settings = {
    .convention = FusionConventionNwu,
    .gain = 0.5f,
    .gyroscopeRange = 250.0f, /* replace this with actual gyroscope range in degrees/s */
    .accelerationRejection = 10.0f,
    .magneticRejection = 10.0f,
    .recoveryTriggerPeriod = 5 * 100 // 100 is supposed to be sample rate SAMPLE_RATE, /* 5 seconds */
};

float RatePitch, RateRoll, RateYaw;
float madwick_roll = 0., madwick_pitch = 0.;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;



struct Receiver_values {
    float pitch = 0;
    float roll = 0;
    float yaw_rate = 0;
    float throttle = 1000;
};

Receiver_values receiver_values;

/* On skip le battery management pour la v1, à faire pour v2
//Battery management variables
float Voltage, Current, BatteryRemaining, BatteryAtStart;
float CurrentConsumed=0;
float BatteryDefault=1300;
*/


// PID Loop variables
int MaxPIDvalues = 300;
    //angle pid
float DesiredRoll, DesiredPitch;
float ErrorRoll, ErrorPitch;
float PrevErrorRoll, PrevErrorPitch;
float PrevItermRoll, PrevItermPitch;
    //rate pid
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
    
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PIDReturn[]={0, 0, 0};

//PID parameters
    //rate
float PRateRoll=1. ; float PRatePitch=PRateRoll; float PRateYaw=0.0; // 0.6 et 2 initially 
float IRateRoll=0.0 ; float IRatePitch=IRateRoll; float IRateYaw=0.; // 3.5 et 12 initially
float DRateRoll=0.0 ; float DRatePitch=DRateRoll; float DRateYaw=0.; //0.03 initially
    //angle
float PRoll= 2. ; float PPitch=PRoll;
float IRoll= 0. ; float IPitch=IRoll;
float DRoll= 0. ; float DPitch=DRoll;


bool flipped;
//motors inputs
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

//timer
uint32_t LoopTimer;

uint32_t lastMpuTime = 0;


void read_mpu(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply low-pass filter to accelerometer data
  filteredAccX = accelFilterAlpha * (a.acceleration.x*0.9953 + 0.02) + (1 - accelFilterAlpha) * filteredAccX;
  filteredAccY = accelFilterAlpha * (a.acceleration.y*0.9983 - 0.067) + (1 - accelFilterAlpha) * filteredAccY;
  filteredAccZ = accelFilterAlpha * (a.acceleration.z*0.9823 - 2.3805) + (1 - accelFilterAlpha) * filteredAccZ;

  // Apply low-pass filter to gyroscope data
  filteredGyroX = gyroFilterAlpha * g.gyro.x + (1 - gyroFilterAlpha) * filteredGyroX;
  filteredGyroY = gyroFilterAlpha * g.gyro.y + (1 - gyroFilterAlpha) * filteredGyroY;
  filteredGyroZ = gyroFilterAlpha * g.gyro.z + (1 - gyroFilterAlpha) * filteredGyroZ;

  // Store filtered values
  AccX = filteredAccX;
  AccY = filteredAccY;
  AccZ = filteredAccZ;

  RateRoll = filteredGyroX;
  RatePitch = filteredGyroY;
  RateYaw = filteredGyroZ;
}

void run_initial_gyro_calibration(){
    //take 2000 measurements before settling errors
    int calibration_measurements = 3000;
    for (RateCalibrationNumber=0; RateCalibrationNumber<calibration_measurements; RateCalibrationNumber ++) {
    read_mpu();
    // make sure data is transferred to rateroll, pitch, yaw
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
    }
    RateCalibrationRoll/= calibration_measurements;
    RateCalibrationPitch/= calibration_measurements;
    RateCalibrationYaw/= calibration_measurements;
}

void read_receiver(){
    if (pulses1 < 2000) {
    receiver_values.throttle = receiver_values.throttle*0.5+ 0.5*pulses1;   
  }

  if (pulses2 < 2000) {
    
    receiver_values.roll = receiver_values.roll*0.2 + 0.8*pulses2;
  }

  if (pulses3 < 2000) {
      receiver_values.pitch = receiver_values.pitch*0.2 + 0.8*pulses3; 
  }

  if (pulses4 < 2000) {
    receiver_values.yaw_rate = receiver_values.yaw_rate*0.2 + 0.8*pulses4;
  }

}

void pulseTimer1() {
  currentTime1 = micros();
  if (currentTime1 > startTime1) {
    pulses1 = currentTime1 - startTime1;
    startTime1 = currentTime1;
  }
}

void pulseTimer2() {
  currentTime2 = micros();
  if (currentTime2 > startTime2) {
    pulses2 = currentTime2 - startTime2;
    startTime2 = currentTime2;
  }
}

void pulseTimer3() {
  currentTime3 = micros();
  if (currentTime3 > startTime3) {
    pulses3 = currentTime3 - startTime3;
    startTime3 = currentTime3;
  }
}

void pulseTimer4() {
  currentTime4 = micros();
  if (currentTime4 > startTime4) {
    pulses4 = currentTime4 - startTime4;
    startTime4 = currentTime4;
  }
}


void initialize_communication(){
    pinMode(RCPin1, INPUT_PULLUP);
    enableInterrupt(RCPin1, pulseTimer1, CHANGE);

    pinMode(RCPin2, INPUT_PULLUP);
    enableInterrupt(RCPin2, pulseTimer2, CHANGE);

    
    pinMode(RCPin3, INPUT_PULLUP);
    enableInterrupt(RCPin3, pulseTimer3, CHANGE);

    pinMode(RCPin4, INPUT_PULLUP);
    enableInterrupt(RCPin4, pulseTimer4, CHANGE);

    
    ESC1.attach(6, 1000, 2000);
    ESC2.attach(7, 1000, 2000);
    ESC3.attach(8, 1000, 2000);
    ESC4.attach(9, 1000, 2000);
}

//void compute_desired_rates(){} designed for rate control

//void compute_rate_errors(){} designed for rate control

void compute_desired_angles(){
    //assuming receiver values between 1000 and 2000 (pmw signal)
    //assuming max_tile of 25 degrees (500*0.05)
    DesiredRoll = 0.05*(receiver_values.roll-1500);
    DesiredPitch = 0.05*(receiver_values.pitch-1500);
    DesiredRateYaw = 0.15*(receiver_values.yaw_rate-1500); // limite rate control to 75 deg/s

    if (3>abs(DesiredRoll)){
      DesiredRoll = 0;
    } 
    if (3>abs(DesiredPitch)){
      DesiredPitch = 0;
    } 


}

void print(){
    Serial.print(receiver_values.throttle);
    Serial.print(" , ");
    Serial.print(DesiredRoll);
    Serial.print(" , ");
    Serial.print(DesiredPitch);
    Serial.print(" , ");
    Serial.print(DesiredRateYaw);
    Serial.print(" , ");
    Serial.print(MotorInput1);
    Serial.print(" , ");
    Serial.print(MotorInput2);
    Serial.print(" , ");
    Serial.print(MotorInput3);
    Serial.print(" , ");
    Serial.print(MotorInput4);
    Serial.print(" , ");
    Serial.print(madwick_roll);
    Serial.print(" , ");
    Serial.print(madwick_pitch);
    Serial.print(" , ");
    Serial.println(RateYaw);
}


void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm){
    double time_passed = (micros() - LoopTimer) / 1000000.0;

    // Proportional term
    float Pterm = P * Error;

    // Integral term
    float Iterm = PrevIterm + I * (Error + PrevError) * time_passed / 2;

    // Anti-windup: Limit the integral term
    float integralMax = MaxPIDvalues / 2.0;
    if (Iterm > integralMax) Iterm = integralMax;
    else if (Iterm < -integralMax) Iterm = -integralMax;

    // Derivative term
    float Dterm = D * (Error - PrevError) / time_passed;

    // Sum PID values
    float PIDOutput = Pterm + Iterm + Dterm;

    // Limit PID output to be within the range of MaxPIDvalues
    if (PIDOutput > MaxPIDvalues) {
        PIDOutput = MaxPIDvalues;
        // Anti-windup: Prevent further increase in Iterm when output is saturated
        Iterm = PrevIterm;
    } else if (PIDOutput < -MaxPIDvalues) {
        PIDOutput = -MaxPIDvalues;
        // Anti-windup: Prevent further decrease in Iterm when output is saturated
        Iterm = PrevIterm;
    }

    // Assign return values
    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}

void apply_PID_loops(){
    //compute angles errror
    ErrorRoll = DesiredRoll- madwick_roll;
    ErrorPitch = DesiredPitch - madwick_pitch;

    //Use PID to compute roll rate and pitch rate (outer control loop)
    pid_equation(ErrorRoll, PRoll, IRoll, DRoll, PrevErrorRoll, PrevItermRoll);
        DesiredRateRoll = PIDReturn[0];
        PrevErrorRoll = PIDReturn[1];
        PrevItermRoll = PIDReturn[2];


    pid_equation(ErrorPitch, PPitch, IPitch, DPitch, PrevErrorPitch, PrevItermPitch);
        DesiredRatePitch = PIDReturn[0];
        PrevErrorPitch = PIDReturn[1];
        PrevItermPitch = PIDReturn[2];

    //Evalutate rates errors
    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;

    //use PID to compute roll, pitch, yaw motor input
    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
       InputRoll=PIDReturn[0];
       PrevErrorRateRoll=PIDReturn[1]; 
       PrevItermRateRoll=PIDReturn[2];

    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
        InputPitch=PIDReturn[0]; 
        PrevErrorRatePitch=PIDReturn[1]; 
        PrevItermRatePitch=PIDReturn[2];

    pid_equation(ErrorRateYaw, PRateYaw,
        IRateYaw, DRateYaw, PrevErrorRateYaw,
        PrevItermRateYaw);
        InputYaw=PIDReturn[0]; 
        PrevErrorRateYaw=PIDReturn[1]; 
        PrevItermRateYaw=PIDReturn[2];
}

void limit_throttle(){
    if (receiver_values.throttle > 1800) receiver_values.throttle = 1800;
}

void reset_pid(){
    PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
    PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
    
    PrevErrorRoll=0; PrevErrorPitch=0;
    PrevItermRoll=0; PrevItermPitch=0; 
}

void apply_calibration(){

    RateRoll-= RateCalibrationRoll;
    RatePitch-= RateCalibrationPitch;
    RateYaw-= RateCalibrationYaw;

}

void compute_motor_inputs(){

    //1. control mixer
    float multiplication_factor = 1.024; // check carbon aeronautics for more info
    MotorInput1 = multiplication_factor*(receiver_values.throttle - InputRoll - InputPitch - InputYaw);
    MotorInput2 = multiplication_factor*(receiver_values.throttle + InputRoll - InputPitch + InputYaw);
    MotorInput3 = multiplication_factor*(receiver_values.throttle + InputRoll + InputPitch - InputYaw);
    MotorInput4 = multiplication_factor*(receiver_values.throttle - InputRoll + InputPitch + InputYaw);

    //2. Limit max and min thrust
    if (MotorInput1 > 2000)MotorInput1 = 1999;
    if (MotorInput2 > 2000)MotorInput2 = 1999; 
    if (MotorInput3 > 2000)MotorInput3 = 1999; 
    if (MotorInput4 > 2000)MotorInput4 = 1999;

    int ThrottleIdle=1200; // minimum spinning throttle //move up?
    if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
    if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
    if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
    if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;

    
    //3. Impose motor cutoff below threshold
        // rest PID values
    int ThrottleCutOff=1000; //cutoff throttle under 5%
    if (receiver_values.throttle < 1050) {
        MotorInput1=ThrottleCutOff; 
        MotorInput2=ThrottleCutOff;
        MotorInput3=ThrottleCutOff; 
        MotorInput4=ThrottleCutOff;
        reset_pid();
    }
    

    if ((flipped) || ((abs(madwick_roll) > 60) || (abs(madwick_pitch) > 60))) {
      flipped = true;
      MotorInput1 = 1000;
      MotorInput2 = 1000;
      MotorInput3 = 1000;
      MotorInput4 = 1000;
    }

    ESC1.writeMicroseconds(MotorInput1);
    ESC2.writeMicroseconds(MotorInput2);
    ESC3.writeMicroseconds(MotorInput3);
    ESC4.writeMicroseconds(MotorInput4);


    
}

void apply_fusion(){
    const FusionVector gyroscope = {57.29577951308232*RateRoll, 57.29577951308232*RatePitch,57.29577951308232*RateYaw}; // replace this with actual gyroscope data in degrees/s
    const FusionVector accelerometer = {AccX/9.81, AccY/9.81, AccZ/9.81}; // replace this with actual accelerometer data in g

    double TimePassed = (micros()-lastMpuTime)/1000000.0;
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, TimePassed);
    lastMpuTime = micros();

    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    madwick_roll = euler.angle.roll;
    madwick_pitch = euler.angle.pitch;

}

void check_mpu_connection(){
  // Try to initialize
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}

void setup(){
    //normal initialization
    //Setup inputs and ouputs, pins etc
    Serial.begin(115200);
    Wire.setClock(400000); //set micro- controller fast mode
    Wire.begin();
    delay(250);

    check_mpu_connection();
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
    Serial.println("Accel set");
    delay(5000);
    run_initial_gyro_calibration();
    FusionAhrsInitialise(&ahrs);

    FusionAhrsSetSettings(&ahrs, &settings);


    /* Set operating frequency? 250 Hz
    analogWriteFrequency(1, 250);
    analogWriteFrequency(2, 250);
    analogWriteFrequency(3, 250);
    analogWriteFrequency(4, 250);
    analogWriteResolution(12);
    */

    initialize_communication();
   //Prevent start of mainloop until after low throttle is positionned
   while ((receiver_values.throttle < 1020) ||  (receiver_values.throttle > 1050)) {
    read_receiver();
    Serial.print("Value : ");
    Serial.print(receiver_values.throttle);
    Serial.println(" , Waiting for data from receiver (low position)");
    read_mpu();
    apply_calibration();
    apply_fusion();
   }

   //Initialize timer
   LoopTimer=micros();
}

void loop(){

    read_mpu();
    apply_calibration();
    apply_fusion();

    read_receiver();

    limit_throttle();

    compute_desired_angles();

    apply_PID_loops();

    //enforce 250 hz frequency, juste after pid_cause thats_where time_passed is used
    while ((micros() - LoopTimer) < 4000){
      LoopTimer = micros();
    }

    print();
    compute_motor_inputs();

}