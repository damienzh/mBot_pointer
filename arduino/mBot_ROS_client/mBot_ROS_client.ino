#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <mbot_pointer/Encoder.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#include <MeAuriga.h>                               

MPU6050 mpu(0x69);
Servo pointer;

//#define OUTPUT_IMU_MEAN
#define OUTPUT_IMU_RAW
//#define DO_CALIBRATION
//#define DEBUG_IMU_RANGE
//#define OUTPUT_COUNT_SPEED
//#define PUBLISH_ENC_SEPERATE
#define DEBUG_RPM_SPEED

int main_delay;
int state=0;
int offset_ax,offset_ay,offset_az,offset_gx,offset_gy,offset_gz;
int buffsize = 10;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
int16_t ax, ay, az, gx, gy, gz;
int enc_reading[2];
float enc_speed[2];
float elapsed;

char frame_id[4] = "imu";

const float rescale = 16384.0 / 9.807;
const float deg2rad = 3.1415926 / 180;
const float count2dis = 0.00035;

long LastPosL, LastPosR, CurPosL, CurPosR;

ros::NodeHandle nh;

std_msgs::Int8 acc_mode;
std_msgs::Int8 gyr_mode;
sensor_msgs::Imu imu_raw;
mbot_pointer::Encoder enc;
ros::Time then, now;
#ifdef PUBLISH_ENC_SEPERATE
std_msgs::Int16 count_left;
std_msgs::Int16 count_right;
#endif
#ifdef OUTPUT_COUNT_SPEED
std_msgs::Float32 speed_left;
std_msgs::Float32 speed_right;
#endif
#ifdef DEBUG_RPM_SPEED
mbot_pointer::Encoder enc_rpm;
#endif

MeEncoderOnBoard Motor_Right(SLOT1);  // right motor     -rpm forward
MeEncoderOnBoard Motor_Left(SLOT2);  // left motor      +rpm forward

ros::Publisher imu_pub("imu_raw", &imu_raw);
#ifdef PUBLISH_ENC_SEPERATE
ros::Publisher Motor_Reading_L("encoder_l", &count_left);
ros::Publisher Motor_Reading_R("encoder_r", &count_right);
#endif
ros::Publisher enc_pub("encoder", &enc);
#ifdef OUTPUT_COUNT_SPEED
ros::Publisher Motor_speed_L("motor_v_l", &speed_left);
ros::Publisher Motor_speed_R("motor_v_r", &speed_right);
#endif
#ifdef DEBUG_RPM_SPEED
ros::Publisher Motor_speed_rpm("motor_rpm", &enc_rpm);
#endif

#ifdef DEBUG_IMU_RANGE
ros::Publisher Acc_Mode("a_mode", &acc_mode);
ros::Publisher Gyro_Mode("g_mode", &gyr_mode);
#endif

void set_speed_rpm(const std_msgs::Int16MultiArray& rpm_cmd) {
  Motor_Left.runSpeed(rpm_cmd.data[0]);
  Motor_Right.runSpeed(rpm_cmd.data[1]);
}

void set_speed_PWM(const std_msgs::Int16MultiArray& pwm_cmd){
  Motor_Left.setTarPWM(pwm_cmd.data[0]);
  Motor_Right.setTarPWM(pwm_cmd.data[1]);
}

void set_pointer(const std_msgs::Int16& angle){
  pointer.write(angle.data);
}

ros::Subscriber<std_msgs::Int16MultiArray> vel_comand_rpm("cmd_vel_rpm", set_speed_rpm);
ros::Subscriber<std_msgs::Int16MultiArray> cmd_vel_PWM("cmd_vel_pwm", set_speed_PWM);
ros::Subscriber<std_msgs::Int16> pointer_angle_subs("pointer_angle", set_pointer);

void isr_process_encoder1(void)
{
  if(digitalRead(Motor_Left.getPortB()) == 0)
  {
    Motor_Left.pulsePosMinus();
  }
  else
  {
    Motor_Left.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(Motor_Right.getPortB()) == 0)
  {
    Motor_Right.pulsePosMinus();
  }
  else
  {
    Motor_Right.pulsePosPlus();
  }
}

//main setup

void setup() {
  // set servo port
  pointer.attach(9); //PORT 4 
  
  // preset imu offset
  offset_ax=-1108; offset_ay=-643; offset_az=5702; 
  offset_gx=47; offset_gy=-9; offset_gz=33;

  Wire.begin();
 
  mpu.initialize();
  
  #ifdef DO_CALIBRATION
  calibrateSensor();
  #endif

  mpu.setXAccelOffset(offset_ax);
  mpu.setYAccelOffset(offset_ay);
  mpu.setZAccelOffset(offset_az);
  mpu.setXGyroOffset(offset_gx);
  mpu.setYGyroOffset(offset_gy);
  mpu.setZGyroOffset(offset_gz); 

  attachInterrupt(Motor_Left.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Motor_Right.getIntNum(), isr_process_encoder2, RISING);
  //Set PWM 8KHz
  setup_PWM();
  set_motor_params();
  
  nh.initNode();
  nh.advertise(imu_pub);  
  nh.advertise(enc_pub);
  #ifdef PUBLISH_ENC_SEPERATE
  nh.advertise(Motor_Reading_L);
  nh.advertise(Motor_Reading_R);
  #endif
  #ifdef OUTPUT_COUNT_SPEED
  nh.advertise(Motor_speed_L);
  nh.advertise(Motor_speed_R);
  #endif
  #ifdef DEBUG_RPM_SPEED
  nh.advertise(Motor_speed_rpm);
  #endif

  nh.subscribe(vel_comand_rpm);
  nh.subscribe(cmd_vel_PWM);
  nh.subscribe(pointer_angle_subs);
  
  #ifdef DEBUG_IMU_RANGE
  nh.advertise(Acc_Mode);
  nh.advertise(Gyro_Mode);
  acc_mode.data = mpu.getFullScaleAccelRange();
  gyr_mode.data = mpu.getFullScaleGyroRange();
  #endif

  while (!nh.connected() ){
    nh.spinOnce();
  }
  #ifdef OUTPUT_IMU_RAW
  nh.loginfo("Output raw imu readings");
  #endif
  #ifdef OUTPUT_IMU_MEAN
  nh.loginfo("Output mean imu readings");
  #endif

  LastPosL = Motor_Left.getCurPos();
  LastPosR = Motor_Right.getCurPos();
  
  then = nh.now();

  //init servo
  pointer.write(90);
}

// main loop

void loop() {
  // read raw accel/gyro measurements from device
  
  #ifdef OUTPUT_IMU_RAW
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  main_delay = 10;
  #endif

  #ifdef OUTPUT_IMU_MEAN
  meanreadings();
  ax = mean_ax; ay = mean_ay; az = mean_az; 
  gx = mean_gx; gy = mean_gy; gz = mean_gz;
  main_delay = 0;
  #endif

  imu_raw.header.stamp=nh.now();
  imu_raw.header.frame_id = frame_id;
  // full range +/- 250 degree/s
  imu_raw.angular_velocity.x = ((float)gx / 131) * deg2rad;
  imu_raw.angular_velocity.y = ((float)gy / 131) * deg2rad;
  imu_raw.angular_velocity.z = ((float)gz / 131) * deg2rad;
  imu_raw.angular_velocity_covariance[0]=0;
  imu_raw.angular_velocity_covariance[4]=0;
  imu_raw.angular_velocity_covariance[8]=0;
  // full range +/- 2g
  //convert unit to m/s²
  imu_raw.linear_acceleration.x = ((float)ax / rescale);
  imu_raw.linear_acceleration.y = ((float)ay / rescale);
  imu_raw.linear_acceleration.z = ((float)az / rescale);
  imu_raw.linear_acceleration_covariance[0]=0;
  imu_raw.linear_acceleration_covariance[4]=0;
  imu_raw.linear_acceleration_covariance[8]=0;
  
  CurPosL = Motor_Left.getCurPos();
  CurPosR = Motor_Right.getCurPos();

  enc_reading[0] = CurPosL - LastPosL;
  enc_reading[1] = CurPosR - LastPosR;

  #ifdef PUBLISH_ENC_SEPERATE
  count_left.data = enc_reading[0];
  count_right.data = -enc_reading[1];
  #endif
  
  now = nh.now();
  enc.header.stamp = now;
  enc.left = enc_reading[0];
  enc.right = -enc_reading[1];
  then = nh.now();

  #ifdef DEBUG_RPM_SPEED
  enc_rpm.header.stamp = nh.now();
  enc_rpm.left = Motor_Left.getCurrentSpeed();
  enc_rpm.right = Motor_Right.getCurrentSpeed();
  #endif

  #ifdef OUTPUT_COUNT_SPEED
  elapsed = now.toSec() - then.toSec();
  nh.loginfo(char(elapsed));
  if (elapsed > 0){
    enc_speed[0] = (float)enc_reading[0] / elapsed;
    enc_speed[1] = (float)enc_reading[1] / elapsed;
    speed_left.data = enc_speed[0] * count2dis;
    speed_right.data = -enc_speed[1] * count2dis;
  }
  else{
    speed_left.data = 0;
    speed_right.data = 0;
  }
  #endif
  
  Motor_Left.loop();
  Motor_Right.loop();
  
  imu_pub.publish(&imu_raw);
  enc_pub.publish(&enc);

  #ifdef PUBLISH_ENC_SEPERATE
  Motor_Reading_L.publish(&count_left);
  Motor_Reading_R.publish(&count_right);
  #endif
  #ifdef OUTPUT_COUNT_SPEED
  Motor_speed_L.publish(&speed_left);
  Motor_speed_R.publish(&speed_right);
  #endif
  #ifdef DEBUG_RPM_SPEED
  Motor_speed_rpm.publish(&enc_rpm);
  #endif

  #ifdef DEBUG_IMU_RANGE
  Acc_Mode.publish(&acc_mode);
  Gyro_Mode.publish(&gyr_mode);
  #endif

  LastPosR = CurPosR;
  LastPosL = CurPosL;

  nh.spinOnce();
  //delay(main_delay);
}

// Used functions

void meanreadings(){
  long buff_ax=0, buff_ay=0, buff_az=0, buff_gx=0, buff_gy=0, buff_gz=0;
  int i = 0;
  while (i<buffsize){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    buff_ax += ax;
    buff_ay += ay;
    buff_az += az;
    buff_gx += gx;
    buff_gy += gy;
    buff_gz += gz;
    i++;
    delay(2);
  }
  mean_ax = buff_ax/buffsize;
  mean_ay = buff_ay/buffsize;
  mean_az = buff_az/buffsize;
  mean_gx = buff_gx/buffsize,
  mean_gy = buff_gy/buffsize;
  mean_gz = buff_gz/buffsize;
}

void setup_PWM(){
  // set PWM 8KHZ
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

void set_motor_params(){
  Motor_Left.setPulse(9);
  Motor_Right.setPulse(9);
  Motor_Left.setRatio(39.267);
  Motor_Right.setRatio(39.267);
  Motor_Left.setPosPid(1.8,0,1.2);
  Motor_Right.setPosPid(1.8,0,1.2);
  Motor_Left.setSpeedPid(2.3,2.6,0.16);
  Motor_Right.setSpeedPid(2.3,2.6,0.16);
}


void calibrateSensor() {
  // reset Offset
  nh.loginfo("Calibrate IMU");
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

}
