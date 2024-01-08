#include<Servo.h>
#include <ArduinoJson.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#define INTERRUPT_PIN 2
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#endif

#define lm35 A8
#define echo 40
#define triq 41
#define KP_mes 0.50
#define KI_mes 0.00
#define KD_mes 200.00
#define KP_roll 0.50
#define KI_roll 0.00
#define KD_roll 200.00
#define KP_pitch 0.50
#define KI_pitch 0.00
#define KD_pitch 200.00
#define KP_yaw 0.50
#define KI_yaw 0.00
#define KD_yaw 200.00
#define IMU_COMMUNICATION_TIMEOUT 700
#define MODE "MODE"
#define THROTTLE "THROTTLE"
#define ROLL "ROLL"
#define PITCH "PITCH"
#define YAW "YAW"
MPU6050 mpu;
Servo ESC1, ESC2, ESC3, ESC4;
Servo servo1, servo2, servo3, servo4;
Servo dif, ontek1, ontek2, arkatek;
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
unsigned long lastmillis=0;
float sicalik_gerilim = 0;
int okunan_degerlm35 = 0;
int state1=3;
float hiz ;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer
int16_t * first_sets;
int16_t * second_sets;
float doluluk;
int okunan_deger =0;

int res_min = 891, res_max = 1977, servo_val1= 0, servo_val2= 90;
long int throttle=0, prevyaw=0, prevpitch=0, prevroll=0, en1=0, en2=0;
float mesafe;
float sure; 
float sicalik = 0;
double roll_pid_i, roll_last_error, roll_control_signal;
double pitch_pid_i, pitch_last_error, pitch_control_signal;
double yaw_pid_i, yaw_last_error, yaw_control_signal;
double mes_pid_i, mes_last_error, mes_control_signal;
int motpower1=0, motpower2=0, motpower3=0, motpower4=0;  

void mot_calib_setup() {
  
  pinMode(triq,OUTPUT);
  pinMode(echo,INPUT);
  
  servo1.attach(2,1000,2000);
  servo2.attach(3,1000,2000);
  servo3.attach(4,1000,2000);
  servo4.attach(5,1000,2000);
  ESC1.attach(6,1000,2000);
  ESC2.attach(7,1000,2000);
  ESC3.attach(8,1000,2000);
  ESC4.attach(9,1000,2000);
  dif.attach(10,1000,2000);
  ontek1.attach(11,1000,2000);
  ontek2.attach(12,1000,2000);
  arkatek.attach(13,1000,2000);
  Serial.println("esc kalibre basladı...");
  ESC1.write(180);
  ESC2.write(180);
  ESC3.write(180);
  ESC4.write(180);
  delay(2000);
  ESC1.write(0);
  ESC2.write(0);
  ESC3.write(0);
  ESC4.write(0); 
  Serial.begin(9600);
  while (!Serial)
    continue;

  
  //nodeMCu üzerinden okunan veri için bekleme süresini
  //arttır(varsayılan=1000)
  Serial.setTimeout(2000);
  delay(2000);
  
  Serial.println("esc kalibre bitti");
}

JsonDocument parseToJsonDoc(String json){
  // Allocate the JSON document
  JsonDocument doc;
  
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, json);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
  }

  return doc;  
}


void resetPidVariablesroll() {
  roll_pid_i = 0;
  roll_last_error = 0;
}

void resetPidVariablesyaw() {
  yaw_pid_i = 0;
  yaw_last_error = 0;
}

void resetPidVariablespitch() {
  pitch_pid_i = 0;
  pitch_last_error = 0;
}

void dmpDataReady() {
  mpuInterrupt = true;
}
unsigned long last_time = 0; 
struct Orientation {
  double YawAngle;
  double PitchAngle;
  double RollAngle;
};
struct IMU_Values {
  bool Error;
  bool NewDataAvailable;
  unsigned long DeltaTime;
  struct Orientation CurrentOrientation;
};

void initializeIMU() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

    mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  if(!mpu.testConnection()){
    Serial.println("*imu test connection failed!");
  }
  devStatus = mpu.dmpInitialize();

  
  mpu.setXAccelOffset(-964.00000);
  mpu.setYAccelOffset(-2713.00000);
  mpu.setZAccelOffset(1634.00000);
  mpu.setXGyroOffset(30);
  mpu.setYGyroOffset(75.00000);
  mpu.setZGyroOffset(-41.00000);
//-964.00000,	-2713.00000,	1634.00000,	30.00000,	75.00000,	-41.00000
//-1012.00000,  1285.00000, 1322.00000, 76.00000, -36.00000,  14.00000
//-560.00000,  -2681.00000,  1680.00000, 35.00000, 75.00000, -39.00000
//3630.00000,	5986.00000,	9086.00000,	156.00000,	30.00000,	19.00000

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
}

struct IMU_Values GetIMU_Values() {
  struct IMU_Values o;
  o.NewDataAvailable = false;
  if (!dmpReady) 
    return o;

  unsigned long current_time = millis();
  unsigned long delta_time = current_time - last_time;
    
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    o.CurrentOrientation.YawAngle = ypr[0] * 180 / M_PI;
    o.CurrentOrientation.RollAngle = ypr[1] * 180 / M_PI * -1; //-1 for changing rotation
    o.CurrentOrientation.PitchAngle = ypr[2] * 180 / M_PI;
    o.NewDataAvailable = true;
    o.DeltaTime = delta_time;
    
    if (last_time == 0){
      last_time = current_time;
      o.Error = true;
      return o;
    }
    last_time = current_time;
  }
  
  if(delta_time > IMU_COMMUNICATION_TIMEOUT){
      o.Error = true;
  }else{
      o.Error = false;
  }

  return o;
}

double getControlSignal(double error, double kp, double ki, double kd, double& pid_i, double& last_error, unsigned long delta_time) {
  double pid_p = error;
  double pid_d = (error - last_error) / delta_time;
  pid_i += error * delta_time;
  
  double control_signal = (kp*pid_p) + (ki*pid_i) + (kd*pid_d);
  last_error = error;
  return control_signal;
}

int calculateMotorPowers(int irtifa,int uzaklik,int PrRoll, int PrPitch, int PrYaw, struct IMU_Values imu_values) {
  double mesError= irtifa - uzaklik;
  Serial.print("mesaferror:");
  Serial.print(mesError);
  Serial.print(",");
  double rollError = PrRoll - imu_values.CurrentOrientation.RollAngle;
  Serial.print("Rollerror:");
  Serial.print(rollError);
  Serial.print(",");
  double yawError = PrYaw - imu_values.CurrentOrientation.YawAngle;
  Serial.print("Yawerror:");
  Serial.print(yawError);
  Serial.print(",");
  double pitchError = PrPitch - imu_values.CurrentOrientation.PitchAngle;
  Serial.print("Pitcherror:");
  Serial.print(pitchError);
  Serial.println(" ");
  mes_control_signal=  getControlSignal(mesError, KP_mes, KI_mes, KD_mes, mes_pid_i, mes_last_error, sure);
  roll_control_signal = getControlSignal(rollError, KP_roll, KI_roll, KD_roll, roll_pid_i, roll_last_error, imu_values.DeltaTime);
  pitch_control_signal = getControlSignal(pitchError, KP_pitch, KI_pitch, KD_pitch, pitch_pid_i, pitch_last_error, imu_values.DeltaTime);
  yaw_control_signal = getControlSignal(yawError, KP_yaw, KI_yaw, KD_yaw, yaw_pid_i, pitch_last_error, imu_values.DeltaTime);
  Serial.print("Yaw:");
  Serial.print(imu_values.CurrentOrientation.YawAngle);
  Serial.print(",");

  Serial.print("Pitch:");
  Serial.print(imu_values.CurrentOrientation.PitchAngle);
  Serial.print(",");

  Serial.print("Roll:");
  Serial.print(imu_values.CurrentOrientation.RollAngle);
  Serial.println();
  
  motpower1 = throttle + mes_control_signal - roll_control_signal - yaw_control_signal + pitch_control_signal;
  if(motpower1<0) motpower1=0;
  if(motpower1>180) motpower1=180;
  Serial.print("mot1:");
  Serial.print(motpower1);
  Serial.println(" ");
  motpower2 = throttle + mes_control_signal + roll_control_signal + yaw_control_signal + pitch_control_signal;
  if(motpower2<0) motpower2=0;
  if(motpower2>180) motpower2=180;
  motpower3 = throttle + mes_control_signal - roll_control_signal + yaw_control_signal - pitch_control_signal;
  if(motpower3<0) motpower3=0;
  if(motpower3>180) motpower3=180;
  motpower4 = throttle + mes_control_signal + roll_control_signal - yaw_control_signal - pitch_control_signal;
  if(motpower4<0) motpower4=0;
  if(motpower4>180) motpower4=180;
  Serial.print("mot4:");
  Serial.print(motpower4);
  Serial.print(" ");
  Serial.print("t:");
  Serial.print(throttle);
  Serial.print(" ");
  Serial.print("mcs:");
  Serial.print(mes_control_signal);
  Serial.print(" ");
  Serial.print("rcs:");
  Serial.print(roll_control_signal);
  Serial.print(" ");
  Serial.print("ycs:");
  Serial.print(yaw_control_signal);
  Serial.print(" ");
  Serial.print("pcs:");
  Serial.print(pitch_control_signal);
  Serial.println(" ");
  return motpower1,motpower2,motpower3,motpower4;
}

float mesafe_yer() {
  
  digitalWrite(triq,LOW);
  delayMicroseconds(2);
  digitalWrite(triq,HIGH);
  delayMicroseconds(10);
  digitalWrite(triq,LOW);
  float x;
  x= sqrt(1 + ( 27 / 273));
  hiz = 10000 / (331 * x);
  sure = pulseIn(echo,HIGH);
  mesafe = sure / hiz  / 2;
  Serial.print("mesafe:");
  Serial.print(mesafe);
  Serial.println(" ");
  return mesafe;
}

void setup(){
  initializeIMU();
  mot_calib_setup();
 
}

void loop() {
  struct IMU_Values imuValues = GetIMU_Values();
  mesafe_yer();

    
    
    if(Serial.available()){

      //wait 2seconds for a string which have escape char '\n'
      //NOTE: if '\n' could not find in 2 seconds, parser will 
      //give error since text will not be complete
          String json=Serial.readStringUntil("\n");

          Serial.println("ayrıştırılacak: "+json);
          JsonDocument doc=parseToJsonDoc(json);

          //write parsed json data to Serial monitor
          state1 = doc[MODE];
           throttle = doc[THROTTLE];
           prevroll = doc[ROLL];
           prevpitch = doc[PITCH];
           prevyaw= doc[YAW];
    }
    
    if()

    if(state1==1) {
          
          
        
          if(imuValues.Error || throttle==0 || prevroll==0 || prevpitch==0 || prevyaw==0) {
            Serial.println(imuValues.Error);
            resetPidVariablesroll();
            resetPidVariablespitch();
            resetPidVariablesyaw();
            return;
          }
          calculateMotorPowers(100,mesafe,0,0,0, imuValues);
          ESC1.write(motpower1);
          ESC2.write(motpower2);
          ESC3.write(motpower3);
          ESC4.write(motpower4);
        }
    else if(state1==2) {
        
        
    //*************************************************************
        
        if(imuValues.Error || throttle==0 || prevroll==0 || prevpitch==0 || prevyaw==0) {
          Serial.println(imuValues.Error);
          resetPidVariablesroll();
          resetPidVariablespitch();
          resetPidVariablesyaw();
          return;
        }
        if(imuValues.NewDataAvailable == false){
          Serial.println(imuValues.NewDataAvailable);
          return;
        }
       

        calculateMotorPowers(100,mesafe,prevroll, prevpitch, prevyaw, imuValues);
        ESC1.write(motpower1);
        ESC2.write(motpower2);
        ESC3.write(motpower3);
        ESC4.write(motpower4);
    }
    else if(state1==3){
      
    //*************************************************************
        
        if(imuValues.Error ) {
          
          Serial.println(imuValues.Error);
          resetPidVariablesroll();
          resetPidVariablespitch();
          resetPidVariablesyaw();
          return;
        }
        if(imuValues.NewDataAvailable == false){
          Serial.println(imuValues.NewDataAvailable);
          return;
        }
        calculateMotorPowers(100,mesafe,0,0,0,imuValues);
        ESC1.write(motpower1);
        ESC2.write(motpower2);
        ESC3.write(motpower3);
        ESC4.write(motpower4);
    }
    else {
      while (mesafe>10) {
        motpower1-=5;
        motpower2-=5;
        motpower3-=5;
        motpower4-=5;
      delay(400);
    }
  }
  
}
