#define Trig_Pin 4
#define Echo_Pin 5

float cm_per_us = 0.01715;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(Trig_Pin, OUTPUT);
  pinMode(Echo_Pin, INPUT);
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

    Serial.println(distance);

    delay(10);
}
