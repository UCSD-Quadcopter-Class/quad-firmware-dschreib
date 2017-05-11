#include "radio.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Simple_AHRS.h>
#include <PID_v1.h>

/* ---------------------------------------------------------
 * SETUP
 * -------------------------------------------------------*/
struct motorObject{
   double setpoint;
   double input;
   double output;
   int pwm;
   /* declare as many members as desired, but the entire structure size must be known to the compiler. */
};

int THROT = 1, throt_val, PWM_THROT = 8, dt, loopTimerCheck=0;
motorObject roll, pitch, yaw;
double Kp=0.2, Ki=0, Kd=0;
double Kp_yaw=0.4, Ki_yaw=0, Kd_yaw=0;
sensors_vec_t calibrationConstants;
sensors_vec_t   orientation_filtered;
long counter = 0, curr_time, old_time;
PID rollPID(&roll.input, &roll.output, &roll.setpoint, Kp, Ki, Kd, DIRECT);
PID pitchPID(&pitch.input, &pitch.output, &pitch.setpoint, Kp, Ki, Kd, DIRECT);
PID yawPID(&yaw.input, &yaw.output, &yaw.setpoint, Kp_yaw, Ki_yaw, Kd_yaw, DIRECT);
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

void receive(void);
void configureLSM9DS0(void);
bool calculateEulerAngles(sensors_vec_t* orientation, sensors_event_t accel_event, sensors_event_t mag_event);


void setup() {
  Serial.begin(115200);
  rfBegin(17);
  roll.setpoint = 0;
  pitch.setpoint = 0;
  yaw.setpoint = 0;
  
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);

  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);

  // Initialise the LSM9DS0 board.
  if(!lsm.begin())
  {
    // There was a problem detecting the LSM9DS0 ... check your connections
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  
  // Setup the sensor gain and integration time.
  configureLSM9DS0();

  sensors_vec_t   orientation_calibration;
  calibrationConstants.roll = 0;
  calibrationConstants.pitch = 0;
  calibrationConstants.heading = 0;
  
  int calibrationCounter = 0;

  while(millis() < 1000){
    if (ahrs.getOrientation(&orientation_calibration))
    {
        calibrationCounter++;
        calibrationConstants.roll += orientation_calibration.roll;
        calibrationConstants.pitch += orientation_calibration.pitch;
        calibrationConstants.heading += orientation_calibration.heading;
        delay(100);
    }
  }
  calibrationConstants.roll  /= calibrationCounter;
  calibrationConstants.pitch /= calibrationCounter;
  calibrationConstants.heading /= calibrationCounter;

  Serial.println(calibrationConstants.roll);
  Serial.println(calibrationConstants.pitch);
  Serial.println(calibrationConstants.heading);
  
  rollPID.SetOutputLimits(-1,1);
  pitchPID.SetOutputLimits(-1,1);
  yawPID.SetOutputLimits(-1,1);
  delay(1000);
}

void loop() {
  
  //receive();

  
  sensors_vec_t   orientation;
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 
  
  //if (ahrs.getOrientation(&orientation))
  if(calculateEulerAngles(&orientation, a, m))
  {
      orientation.roll -= calibrationConstants.roll;
      orientation.pitch -= calibrationConstants.pitch;
      orientation.heading -= calibrationConstants.heading;
      if(counter = 0){
        curr_time = millis();
        int old_time = curr_time;
        orientation_filtered.roll = orientation.roll;
        orientation_filtered.pitch = orientation.pitch;
        orientation_filtered.heading = orientation.heading;
      }
      else{
        curr_time = millis();
        dt = curr_time - old_time;
        orientation_filtered.pitch = (orientation_filtered.pitch + g.gyro.y*dt/1000*(-1))*.98 + orientation.pitch * 0.02;
        orientation_filtered.roll = (orientation_filtered.roll + g.gyro.x*dt/1000*(-1))*.98 + orientation.roll * 0.02;
        orientation_filtered.heading = (orientation_filtered.roll + g.gyro.z*dt/1000*(-1))*.98 + orientation.heading * 0.02;
        old_time = curr_time;
      }
      
      counter ++;
  }
  
  pitch.input = orientation_filtered.pitch;
  roll.input = orientation_filtered.roll;
  yaw.input = orientation_filtered.heading;

  pitchPID.Compute();
  rollPID.Compute();
  yawPID.Compute();

  int motor1PWM = (int)(127.5-((double)pitch.output)*100);
  int motor2PWM = (int)(127.5+((double)pitch.output)*100);
  analogWrite(8,motor1PWM);
  analogWrite(9,motor2PWM);
  
  /*
  Serial.print(orientation_filtered.pitch); Serial.print(" "); Serial.print(millis());
  Serial.print(" "); Serial.print(pitch.output);
  Serial.print(" "); Serial.print(motor1PWM);
  Serial.print(" "); Serial.print(motor2PWM);
  Serial.print(" "); Serial.print(pitch.output);
  Serial.print(" "); Serial.println(dt);
  */
  int timerCheck = millis();
  Serial.println(timerCheck-loopTimerCheck);
  loopTimerCheck = timerCheck;

 
  
  delay(1);
}

void receive() {
  uint8_t pwm[4], *gim_ptr, count=0;
  gim_ptr = pwm;

  //read until a complete packet
 while (count<4) {
    count=rfRead(gim_ptr,4);
    //Serial.print("Count: ");
    //Serial.println(count);
  }

  Serial.println("Start");
  Serial.println(pwm[0]);
  Serial.println(pwm[1]);
  Serial.println(pwm[2]);
  Serial.println(pwm[3]);

  //Throttle
  analogWrite(8,(pwm[1]/(float)255)*(pwm[2]));
  analogWrite(9,(pwm[1]/(float)255)*(280-pwm[2]));

}


void configureLSM9DS0(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

bool calculateEulerAngles(sensors_vec_t* orientation, sensors_event_t accel_event, sensors_event_t mag_event)
{

  //slighlty modified adafruit ahrs library function
  
  if (orientation == NULL) return false;
  float const PI_F = 3.14159265F;
  // roll: Rotation around the X-axis. -180 <= roll <= 180                                          
  // a positive roll angle is defined to be a clockwise rotation about the positive X-axis          
  //                                                                                                
  //                    y                                                                           
  //      roll = atan2(---)                                                                         
  //                    z                                                                           
  //                                                                                                
  // where:  y, z are returned value from accelerometer sensor                                      
  orientation->roll = (float)atan2(accel_event.acceleration.y, accel_event.acceleration.z);

  // pitch: Rotation around the Y-axis. -180 <= roll <= 180                                         
  // a positive pitch angle is defined to be a clockwise rotation about the positive Y-axis         
  //                                                                                                
  //                                 -x                                                             
  //      pitch = atan(-------------------------------)                                             
  //                    y * sin(roll) + z * cos(roll)                                               
  //                                                                                                
  // where:  x, y, z are returned value from accelerometer sensor                                   
  if (accel_event.acceleration.y * sin(orientation->roll) + accel_event.acceleration.z * cos(orientation->roll) == 0)
    orientation->pitch = accel_event.acceleration.x > 0 ? (PI_F / 2) : (-PI_F / 2);
  else
    orientation->pitch = (float)atan(-accel_event.acceleration.x / (accel_event.acceleration.y * sin(orientation->roll) + \
                                                                     accel_event.acceleration.z * cos(orientation->roll)));

  // heading: Rotation around the Z-axis. -180 <= roll <= 180                                       
  // a positive heading angle is defined to be a clockwise rotation about the positive Z-axis       
  //                                                                                                
  //                                       z * sin(roll) - y * cos(roll)                            
  //   heading = atan2(--------------------------------------------------------------------------)  
  //                    x * cos(pitch) + y * sin(pitch) * sin(roll) + z * sin(pitch) * cos(roll))   
  //                                                                                                
  // where:  x, y, z are returned value from magnetometer sensor                                    
  orientation->heading = (float)atan2(mag_event.magnetic.z * sin(orientation->roll) - mag_event.magnetic.y * cos(orientation->roll), \
                                      mag_event.magnetic.x * cos(orientation->pitch) + \
                                      mag_event.magnetic.y * sin(orientation->pitch) * sin(orientation->roll) + \
                                      mag_event.magnetic.z * sin(orientation->pitch) * cos(orientation->roll));


  // Convert angular data to degree 
  orientation->roll = orientation->roll * 180 / PI_F;
  orientation->pitch = orientation->pitch * 180 / PI_F;
  orientation->heading = orientation->heading * 180 / PI_F;

  return true;
  
}

