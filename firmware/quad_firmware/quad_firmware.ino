int THROT = 1, throt_val;
int PWM_THROT = 8, pwm;

void setup() {
  Serial.begin(9600);
  pinMode(8, OUTPUT);
}

void loop() {
  throt_val = analogRead(THROT);

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
  
  delay(10);
}
