#include "radio.h"

void receive();

int THROT = 1, throt_val;
int PWM_THROT = 8, pwm;

void setup() {
  Serial.begin(38400);
  pinMode(8, OUTPUT);
  rfBegin(17);
}

void loop() {
  
  receive();
  
  /*throt_val = analogRead(THROT);

  if ((throt_val-115)*2 < 50) {
    pwm = 0;
  }
  else if ((throt_val-115)*2 > 1420) {
    pwm = 255;
  }
  else {
    pwm = ((float)(throt_val-115)*2/1420)*255;
  }

  analogWrite(8,pwm);
  Serial.println(pwm);
  */
  delay(10);
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
  analogWrite(8,pwm[1]);
  
}

