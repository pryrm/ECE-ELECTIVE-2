// Motor pins (Driver 1 and Driver 2)
int pinlist[] = {PB1, PA10, PA9, PA8, PB9, PB8, PB7, PB6};
volatile int counter_isr = 0;  

void moveForward() {
  digitalWrite(PB1, HIGH);  digitalWrite(PA10, LOW); // Motor 1
  digitalWrite(PA9, HIGH);  digitalWrite(PA8, LOW);  // Motor 2
  digitalWrite(PB9, HIGH);  digitalWrite(PB8, LOW);  // Motor 3
  digitalWrite(PB7, HIGH);  digitalWrite(PB6, LOW);  // Motor 4
}

void stopAllMotors() {
  for (int i = 0; i < sizeof(pinlist)/sizeof(pinlist[0]); i++) {
    digitalWrite(pinlist[i], LOW);
  }
}

void counter_func() {
  counter_isr++;
}

void setup() {
  // Setup motor pins
  for (int i = 0; i < sizeof(pinlist)/sizeof(pinlist[0]); i++) {
    pinMode(pinlist[i], OUTPUT);
    digitalWrite(pinlist[i], LOW);
  }

  // Setup interrupt pin
  pinMode(PA3, INPUT_PULLUP);  
  attachInterrupt(digitalPinToInterrupt(PA3), counter_func, RISING);  // Correct attachInterrupt usage
}

void loop() {
  if (counter_isr < 91) { 
    moveForward();
  } else {
    stopAllMotors();
    // Optional: prevent running stopAllMotors() repeatedly
    while (1);  // halt execution
  }
}
