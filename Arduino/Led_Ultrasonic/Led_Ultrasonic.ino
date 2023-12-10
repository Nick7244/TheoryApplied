#define LED_Pin 3
#define Trig_Pin 4
#define Echo_Pin 5

float cm_per_us = 0.01715;
int fsm_state;

enum FSM_STATES {
    UNDETECTED  = 0,
    DETECTED    = 1,
    LED_ON      = 2,
    LED_OFF     = 3
};

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(Trig_Pin, OUTPUT);
    pinMode(Echo_Pin, INPUT);
    pinMode(LED_Pin, OUTPUT);

    fsm_state = UNDETECTED;
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
    Serial.println(fsm_state);
    Serial.println();

    switch (fsm_state)
    {
        case UNDETECTED:
            if (distance <= 200.0)
            {
                fsm_state = DETECTED;
            }
            break;
        
        case DETECTED:
        {
            int pwm = 0;
            int startTime = millis();
            
            double p = 0.1;
            double tf = 1000.0; // 1000 ms to completion
            auto r = log((5+p)/p) / tf;

            while (true)
            {
                int t = millis() - startTime;
                auto desiredVoltage = p * exp(r * double(t)) - p;
                int pwm = round(desiredVoltage * (255.0/5.0));

                if (pwm > 255) { pwm = 255; }
                Serial.println(pwm);

                analogWrite(LED_Pin, pwm);

                if (pwm >= 255) {break;}
                delay(1);
            }

            fsm_state = LED_ON;            
            break;
        }

        case LED_ON:
            if (distance > 200.0)
            {
                // analogWrite(LED_Pin, 0);
                // fsm_state = UNDETECTED;
            }
            break;

        case LED_OFF:

        default:
            // do nothing
            break;
    };

    delay(100);
}
