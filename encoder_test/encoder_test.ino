byte PWM_PIN = 3;
 
int pwm_value;
 
void setup() {
  pinMode(PWM_PIN, INPUT);
  Serial.begin(115200);
}
 
void loop() {
  pwm_value = analogRead(PWM_PIN);
  Serial.println(pwm_value);
}
