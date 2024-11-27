#define POWER_DETECT_PIN A0
#define ENGAGE_A4988_PIN 2
#define DIR_PIN 7
#define STEP_PIN 8

int count = 0;

void setup()
{
  Serial.begin(9600);
  
  // Establish engage pin and set to HIGH to disable a4988
  pinMode(ENGAGE_A4988_PIN, OUTPUT);
  digitalWrite(ENGAGE_A4988_PIN, HIGH);

  // Establish power detection pin
  pinMode(POWER_DETECT_PIN, INPUT);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  digitalWrite(DIR_PIN, HIGH);

}

void loop()
{  
  int powerDetection = analogRead(POWER_DETECT_PIN);
  // Serial.println(powerDetection);
  
  if (powerDetection > 750)
  {
    // Serial.println("POWER DETECTED, ENGAGING MOTOR DRIVER");
    digitalWrite(ENGAGE_A4988_PIN, LOW);

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(5000);
    count++;
    // Serial.println(count);
    if (count > 200)
    {
      count = 0;
    }
  }
  else
  {
    Serial.println("POWER NOT DETECTED, DISENGAGING MOTOR DRIVER");
    digitalWrite(ENGAGE_A4988_PIN, HIGH);

    delay(100);
  }
}
