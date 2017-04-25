#include <serLCD.h>
int YAW = 0, yaw_val;
int THROT = 1, throt_val;
int ROLL = 2, roll_val;
int PITCH = 3, pitch_val;


serLCD lcd;

void setup() {
  Serial.begin(9600);
  lcd.clear();
  lcd.setBrightness(1);
}

void loop() {
  yaw_val = analogRead(YAW);
  throt_val = analogRead(THROT);
  roll_val = analogRead(ROLL);
  pitch_val = analogRead(PITCH);

  lcd.setCursor(0,0);
  lcd.print("Y:");
  lcd.print((yaw_val-115)*2);
  lcd.setCursor(0,8);
  lcd.print("T:");
  lcd.print((throt_val-115)*2);

  lcd.setCursor(1,0);
  lcd.print("R:");
  lcd.print((roll_val-115)*2);
  lcd.setCursor(1,8);
  lcd.print("P:");
  lcd.print((pitch_val-115)*2);

  
  /* Printing to serial monitor
  Serial.print(" Yaw: ");
  Serial.print((yaw_val-115)*2);
  Serial.print(" Throttle: ");
  Serial.print((throt_val-115)*2);
  Serial.print(" Roll; ");
  Serial.print((roll_val-115)*2);
  Serial.print(" Pitch: ");
  Serial.println((pitch_val-115)*2);
  */
  
}
