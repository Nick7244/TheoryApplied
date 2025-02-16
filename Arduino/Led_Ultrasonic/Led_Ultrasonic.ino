#define Trig_Pin 10//blue
#define Echo_Pin 12 //yellow
#define Button_Pin 9 
#define LED_Pin 3

float cm_per_us = 0.01715;
int fsmState_;
int stateCounter_;

enum fsmStateS {
  UNDETECTED  = 0,
  DETECTED    = 1,
  LED_ON      = 2,
  LED_OFF     = 3
};

void setup() {
  Serial.begin(9600);
  pinMode(Trig_Pin, OUTPUT);
  pinMode(Echo_Pin, INPUT);
  pinMode(LED_Pin, OUTPUT);
  pinMode(Button_Pin, INPUT);

  // Initialize FSM
  fsmState_ = UNDETECTED;
  stateCounter_ = 0;
}

void loop() {
  // Execute ultrasonic 'chirp'
  digitalWrite(Trig_Pin, LOW);
  delayMicroseconds(10);
  digitalWrite(Trig_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_Pin, LOW);

  // Wait for echo pin to read signal
  while (digitalRead(Echo_Pin) == LOW) {};

  // Compute time of flight
  auto startTime = micros();
  while (digitalRead(Echo_Pin) == HIGH)
  {   
    // Maximum of 17500us --> ~300cm
    if (micros() - startTime > 17500) { break; }
  };

  auto twoWayTimeOfFlight_us = (micros() - startTime);

  // Compute distance in cm
  float distance = cm_per_us * float(twoWayTimeOfFlight_us);

  Serial.print("FSM State: ");
  Serial.print(fsmState_);
  Serial.print(", State counter = ");
  Serial.print(stateCounter_);
  Serial.print(", Distance (cm) = ");
  Serial.println(distance);
  Serial.println();

  updateFSMTime(&stateCounter_);

  switch (fsmState_)
  {
    case UNDETECTED: 
    {
      //Reset distance counter upon entry to state
      static int distanceCounter = 0;
      if(stateCounter_ == 1)
      {
        distanceCounter = 0;
      }

      //Update distance counter
      if(distance <= 150.0)
      {
        distanceCounter++;
      }
      else
      {
        distanceCounter = 0;
      }

      //If we're certain there is someone there, turn the LED on
      if(distanceCounter >= 10)
      {
        updateFSMState(&fsmState_, &stateCounter_, DETECTED);
      }
      break;
    }
    
    case DETECTED:
    {
      int pwm = 0;
      
      // Parameters defining exponential growth of LED voltage
      // (leads to pseudo-linear brightness growth)
      double p = 0.1;
      double tf = 1000.0; // 1000 ms to completion
      auto r = log((5+p)/p) / tf;

      int startTime = millis();

      // Turn on LED
      while (true)
      {
        int t = millis() - startTime;
        auto desiredVoltage = p * exp(r * double(t)) - p;
        int pwm = round(desiredVoltage * (255.0/5.0));

        if (pwm > 255) { pwm = 255; }
        Serial.print(pwm);
        Serial.print(", ");
        Serial.print(p);
        Serial.print(", ");
        Serial.print(r * 1000.0);
        Serial.print(", ");
        Serial.print(t);
        Serial.print(", ");
        Serial.println(desiredVoltage);

        analogWrite(LED_Pin, pwm);

        if (pwm >= 255) {break;}
        delay(1);
      }

      updateFSMState(&fsmState_, &stateCounter_, LED_ON);              
      break;
    }

    case LED_ON:
      // Once button is pressed, turn LED off
      if (digitalRead(Button_Pin) == HIGH)
      {
        analogWrite(LED_Pin, 0);
        updateFSMState(&fsmState_, &stateCounter_, LED_OFF);    
      }
      break;

    case LED_OFF:
    {
      //Reset distance counter upon entry to state
      static int distanceCounter = 0;
      if(stateCounter_ == 1)
      {
        distanceCounter = 0;
      }

      // Wait until obstacle is gone to reset FSM
      if (distance > 200.0)
      {
        distanceCounter++;
      }
      else
      {
        distanceCounter = 0;
      }

      //If we're certain the person is gone, 
      if(distanceCounter >= 10 && stateCounter_ >= 50)
      {
        updateFSMState(&fsmState_, &stateCounter_, UNDETECTED);
      }
      
      break;
    }

    default:
      // do nothing
      break;
  };

  delay(100);
}


void updateFSMState(int * fsmState, int * stateCounter, int newState)
{
  *fsmState = newState;
  *stateCounter = 0;
}

void updateFSMTime(int * stateCounter)
{
  *stateCounter += 1;
}
