#define POWER_DETECT_PIN A0
#define ENGAGE_A4988_PIN 2
#define DIR_PIN 7
#define STEP_PIN 8
#define POT_PIN A1

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

  pinMode(POT_PIN, INPUT);
}

void loop()
{  
  int powerDetection = analogRead(POWER_DETECT_PIN);
  // Serial.println(powerDetection);
  int potVal = analogRead(POT_PIN);

  int delayVal;
  if(potVal > 580)
  {
    digitalWrite(DIR_PIN, HIGH);
    delayVal = 2000.0 - (1700.0/514.0)*float(potVal - 580);
    // Serial.println("HIGH + " + String(delayVal));
  }
  else if(potVal < 450)
  {
    digitalWrite(DIR_PIN, LOW);
    delayVal = (1700.0/514.0)*float(potVal) + 500;
    // Serial.println("LOW + " + String(delayVal));
  }
  else
  {
    digitalWrite(STEP_PIN, LOW);
    delayVal = 0;
    // Serial.println("STOPPING MOTOR + " + String(delayVal));
  }
  
  if (powerDetection > 700 && delayVal > 0)
  {
    // Serial.println("POWER DETECTED, ENGAGING MOTOR DRIVER");
    digitalWrite(ENGAGE_A4988_PIN, LOW);

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500);
    // count++;
    // Serial.println(count);
    // if (count > 200)
    // {
    //   count = 0;
    // }
  }
  else
  {
    Serial.println("POWER NOT DETECTED, DISENGAGING MOTOR DRIVER");
    digitalWrite(ENGAGE_A4988_PIN, HIGH);

    delay(100);
  }
}
