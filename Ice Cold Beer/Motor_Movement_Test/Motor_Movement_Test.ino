#define POWER_DETECT_PIN A0
#define ENGAGE_A4988_PIN 2
#define DIR_PIN 7
#define STEP_PIN 8
#define POT_PIN A1

int count = 0;
unsigned long loopStartTime = micros();
unsigned long oneSecondTimerStartTime;
unsigned long quarterSecondTimerStartTime;
unsigned long averageLoopTime;
int averageLoopTimeCounter = 0;

unsigned long sleepTime;
unsigned long motorOnTime;

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
  oneSecondTimerStartTime = millis();
  quarterSecondTimerStartTime = millis();
}

void loop()
{  
  unsigned long loopTime = micros() - loopStartTime;
  loopStartTime = micros();

  bool oneSecondTimerEnded = (millis() - oneSecondTimerStartTime) > 1000;
  bool quarterSecondTimeEnded = (millis() - quarterSecondTimerStartTime) > 250;
  
  if(oneSecondTimerEnded)
  {
    oneSecondTimerStartTime = millis();
  }
  
  if(quarterSecondTimeEnded)
  {
    quarterSecondTimerStartTime = millis();
  }

  averageLoopTime += loopTime;
  averageLoopTimeCounter++;

  if(oneSecondTimerEnded)
  {
    // Serial.println("Average loop tims (us): " +
      // String((float)(averageLoopTime) / (float)(averageLoopTimeCounter)));
    averageLoopTime = 0;
    averageLoopTimeCounter = 0;
  }
  
  int powerDetection = analogRead(POWER_DETECT_PIN);
  if(oneSecondTimerEnded)
  {
    // Serial.println(powerDetection);
  }
  int potVal = analogRead(POT_PIN);

  int delayVal;
  if(potVal > 580)
  {
    digitalWrite(DIR_PIN, HIGH);
    delayVal = 2000.0 - (1700.0/514.0)*float(potVal - 580);
    
    if(quarterSecondTimeEnded)
    {
      // Serial.println("HIGH + " + String(delayVal));
    }
  }
  else if(potVal < 450)
  {
    digitalWrite(DIR_PIN, LOW);
    delayVal = (1700.0/514.0)*float(potVal) + 500;

    if(quarterSecondTimeEnded)
    {
      // Serial.println("LOW + " + String(delayVal));
    }
  }
  else
  {
    digitalWrite(STEP_PIN, LOW);
    delayVal = 0;
    
    if(quarterSecondTimeEnded)
    {
      // Serial.println("STOPPING MOTOR + " + String(delayVal));
    }
  }
  
  if (powerDetection > 700 && delayVal > 0)
  {
    if(oneSecondTimerEnded)
    {
      // Serial.println("POWER DETECTED, ENGAGING MOTOR DRIVER");
    }
    digitalWrite(ENGAGE_A4988_PIN, LOW);

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delayVal);
    digitalWrite(STEP_PIN, LOW);
    
    sleepTime = delayVal;
    motorOnTime = delayVal;
  }
  else
  {
    if(oneSecondTimerEnded)
    {
      // Serial.println("POWER NOT DETECTED, DISENGAGING MOTOR DRIVER");
    }

    digitalWrite(ENGAGE_A4988_PIN, HIGH);
    sleepTime = 1000;
    motorOnTime = 0;
  }

  if(oneSecondTimerEnded)
  {
    // Serial.println(" ");
  }

  unsigned long timeElapsed = micros() - loopStartTime;
  unsigned long nonMotorOnCodeTime = timeElapsed - motorOnTime;
  if(oneSecondTimerEnded)
  {
    // Serial.println(nonMotorOnCodeTime);
    // Serial.println(sleepTime);
    // Serial.println(sleepTime - nonMotorOnCodeTime - 25);
  }
  delayMicroseconds(sleepTime - nonMotorOnCodeTime - 25);
}
