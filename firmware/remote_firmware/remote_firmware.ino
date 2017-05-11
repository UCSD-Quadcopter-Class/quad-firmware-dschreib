#include <serLCD.h>
#include "radio.h"

void lcd_print(int yaw_val,int throt_val,int roll_val,int pitch_val);
void send_vals(int yaw_val,int throt_val,int roll_val,int pitch_val);

int YAW = 0, yaw_val;
int THROT = 1, throt_val;
int ROLL = 2, roll_val;
int PITCH = 3, pitch_val;

serLCD lcd;

void setup() {
  Serial.begin(38400);
  //Serial1.begin(115200);
  rfBegin(17);
  lcd.clear();
  lcd.setBrightness(1);
}

void loop() {

  //Printing gimbal values to lcd screen
  yaw_val = analogRead(YAW);
  throt_val = analogRead(THROT);
  roll_val = analogRead(ROLL);
  pitch_val = analogRead(PITCH);

  //Converting to values between 0-1500
  yaw_val = (yaw_val-115)*2;
  throt_val = (throt_val-115)*2;
  roll_val = (roll_val-115)*2;
  pitch_val = (pitch_val-115)*2;
  
  lcd_print(yaw_val,throt_val,roll_val,pitch_val);
  send_vals(yaw_val,throt_val,roll_val,pitch_val);
  
  delay(10);
}

void lcd_print(int yaw_val,int throt_val,int roll_val,int pitch_val) {
  char text[4];
  lcd.setCursor(1,8);
  lcd.print("P:");
  sprintf(text,"%4d",pitch_val);
  lcd.print(text);
  
  lcd.setCursor(0,0);
  lcd.print("Y:");
  sprintf(text,"%4d",yaw_val);
  lcd.print(text);

  lcd.setCursor(0,8);
  lcd.print("T:");
  sprintf(text,"%4d",throt_val);
  lcd.print(text);

  lcd.setCursor(1,0);
  lcd.print("R:");
  sprintf(text,"%4d",roll_val);
  lcd.print(text);
}

void send_vals(int yaw_val,int throt_val,int roll_val,int pitch_val) {
  uint8_t pwm[4], *gim_ptr;
  gim_ptr = pwm;
  int gim_vals[4];
  gim_vals[0] = yaw_val;
  gim_vals[1] = throt_val;
  gim_vals[2] = roll_val;
  gim_vals[3] = pitch_val;
  
  for (int i=0; i<4; i++) {
    if (gim_vals[i] < 50 ) {
      pwm[i] = 0;
    }
    else if (gim_vals[i] > 1400) {
      pwm[i] = 255;
    }
    else {
      pwm[i] = ((float)(gim_vals[i])/1400)*255;
    }
    //Serial.println(pwm[i]);
  }

  rfWrite(gim_ptr, 4);
  //Serial.println("new readings");
  
}


