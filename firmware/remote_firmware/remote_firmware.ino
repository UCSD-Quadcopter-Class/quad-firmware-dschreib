#include <serLCD.h>
int YAW = 0, yaw_val;
int THROT = 1, throt_val;
int ROLL = 2, roll_val;
int PITCH = 3, pitch_val;
char text[4];

serLCD lcd;

void setup() {
  Serial.begin(9600);
  lcd.clear();
  lcd.setBrightness(1);
}

void loop() {

  //Printing gimbal values to lcd screen
  yaw_val = analogRead(YAW);
  throt_val = analogRead(THROT);
  roll_val = analogRead(ROLL);
  pitch_val = analogRead(PITCH);
  
  lcd.setCursor(1,8);
  lcd.print("P:");
  sprintf(text,"%4d",((pitch_val-115)*2));
  lcd.print(text);
  
  lcd.setCursor(0,0);
  lcd.print("Y:");
  sprintf(text,"%4d",((yaw_val-115)*2));
  lcd.print(text);

  lcd.setCursor(0,8);
  lcd.print("T:");
  sprintf(text,"%4d",(throt_val-115)*2);
  lcd.print(text);

  lcd.setCursor(1,0);
  lcd.print("R:");
  sprintf(text,"%4d",(roll_val-115)*2);
  lcd.print(text);
  
  /*
  //Printing  gimbal values to serial monitor
  Serial.print(" Yaw: ");
  Serial.print((yaw_val-115)*2);
  Serial.print(" Throttle: ");
  Serial.print((throt_val-115)*2);
  Serial.print(" Roll; ");
  Serial.print((roll_val-115)*2);
  Serial.print(" Pitch: ");
  Serial.println((pitch_val-115)*2);*/
  
  delay(10);
}
