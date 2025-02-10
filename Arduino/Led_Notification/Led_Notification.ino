#define RED_LED 3
#define GREEN_LED 4
#define BUTTON_1  7
#define BUTTON_2  8

enum LED_STATES
{
  BOTH_OFF  = 0,
  GREEN_ON    = 1,
  RED_ON      = 2
};

int fsm_state;

void setup() 
{
  Serial.begin(9600);
  pinMode(BUTTON_1, INPUT);
  pinMode(BUTTON_2, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  // Initialize FSM
  fsm_state = BOTH_OFF;
}

void loop()
{
  // put your main code here, to run repeatedly:

  switch(fsm_state)
  {
    case BOTH_OFF:
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, LOW);

      if (digitalRead(BUTTON_1) == HIGH)
      {
        fsm_state = GREEN_ON;        
      }
      else if (digitalRead(BUTTON_2) == HIGH)
      {
        fsm_state = RED_ON;        
      }

      break;
    
    case GREEN_ON:
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);

      if (digitalRead(BUTTON_2) == HIGH)
      {
        fsm_state = RED_ON;        
      }
      break;
    
    case RED_ON:
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);

      if (digitalRead(BUTTON_1) == HIGH)
      {
        fsm_state = GREEN_ON;        
      }
      break;
    
    default:
      break;
  }

  Serial.println(fsm_state);
  delay(100);
}
