void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(3, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  unsigned long t = millis();
  float freq = 0.5;
  float desiredVoltage = 2.5*sin(2*3.1415*freq*t/1000.0) + 2.5;
  int pwm = round(desiredVoltage * (255.0/5.0));
  analogWrite(3, pwm);
  delay(10);
}
