// STM32-based Obstacle Avoidance Robot

#include <Servo.h>

// Motor pins (Driver 1 and Driver 2)
int pinlist[] = {PB1, PA10, PA9, PA8, PB9, PB8, PB7, PB6};

// Ultrasonic sensor
const int trig = PA0;
const int echo = PA1;

// Servo
Servo myservo;
const int servoPin = PA2;

// Motor PWM states
typedef struct {
  int pin_on;
  int pin_off;
  bool inverse;
  int freq;
  float duty;
  unsigned long period_us;
  unsigned long on_time_us;
  unsigned long last_toggle_time;
  bool state;
} PWMState;

PWMState motors[] = {
  {PB1, PA10, false, 20, 100.0, 0, 0, 0, false}, // Motor 1
  {PA9, PA8, false, 20, 100.0, 0, 0, 0, false},  // Motor 2
  {PB9, PB8, false, 20, 100.0, 0, 0, 0, false},  // Motor 3
  {PB7, PB6, false, 20, 100.0, 0, 0, 0, false}   // Motor 4
};

void sendTriggerPulse() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
}

uint32_t pulseInSTM(uint8_t pin, uint8_t state, uint32_t timeout_us = 23529) {
  uint32_t startMicros = 0, endMicros = 0;
  uint8_t oppositeState = !state;

  uint32_t start = micros();
  while (digitalRead(pin) == state) {
    if (micros() - start > timeout_us) return 0;
  }

  start = micros();
  while (digitalRead(pin) == oppositeState) {
    if (micros() - start > timeout_us) return 0;
  }

  startMicros = micros();
  while (digitalRead(pin) == state) {
    if (micros() - startMicros > timeout_us) return 0;
  }

  endMicros = micros();
  return endMicros - startMicros;
}

void initPWM(PWMState* pwm) {
  pwm->period_us = 1000000.0 / pwm->freq;
  pwm->on_time_us = pwm->period_us * (pwm->duty / 100.0);
  pwm->last_toggle_time = micros();
  pwm->state = false;
}

void updatePWM(PWMState* pwm) {
  unsigned long now = micros();
  unsigned long elapsed = now - pwm->last_toggle_time;

  if (!pwm->state) {
    if (elapsed >= (pwm->period_us - pwm->on_time_us)) {
      pwm->last_toggle_time = now;
      pwm->state = true;
      digitalWrite(pwm->pin_on, HIGH);
    }
  } else {
    if (elapsed >= pwm->on_time_us) {
      pwm->last_toggle_time = now;
      pwm->state = false;
      digitalWrite(pwm->pin_on, LOW);
    }
  }
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(motors[i].pin_on, LOW);
    digitalWrite(motors[i].pin_off, LOW);
  }
}

void moveForward() {
  digitalWrite(PB1, HIGH);  digitalWrite(PA10, LOW); // Motor 1
  digitalWrite(PA9, HIGH);  digitalWrite(PA8, LOW);  // Motor 2
  digitalWrite(PB9, HIGH);  digitalWrite(PB8, LOW);  // Motor 3
  digitalWrite(PB7, HIGH);  digitalWrite(PB6, LOW);  // Motor 4
}

void moveBackward() {
  digitalWrite(PB1, LOW);   digitalWrite(PA10, HIGH); // Motor 1
  digitalWrite(PA9, LOW);   digitalWrite(PA8, HIGH);  // Motor 2
  digitalWrite(PB9, LOW);   digitalWrite(PB8, HIGH);  // Motor 3
  digitalWrite(PB7, LOW);   digitalWrite(PB6, HIGH);  // Motor 4
}

void turnLeft() {
  digitalWrite(PB1, LOW);   digitalWrite(PA10, HIGH); // Motor 1 backward
  digitalWrite(PA9, HIGH);  digitalWrite(PA8, LOW);   // Motor 2 forward
  digitalWrite(PB9, LOW);   digitalWrite(PB8, HIGH);  // Motor 3 backward
  digitalWrite(PB7, HIGH);  digitalWrite(PB6, LOW);   // Motor 4 forward
}

void turnRight() {
  digitalWrite(PB1, HIGH);  digitalWrite(PA10, LOW);  // Motor 1 forward
  digitalWrite(PA9, LOW);   digitalWrite(PA8, HIGH);  // Motor 2 backward
  digitalWrite(PB9, HIGH);  digitalWrite(PB8, LOW);   // Motor 3 forward
  digitalWrite(PB7, LOW);   digitalWrite(PB6, HIGH);  // Motor 4 backward
}

void setup() {
  // Setup motors
  for (int i = 0; i < sizeof(pinlist)/sizeof(pinlist[0]); i++) {
    pinMode(pinlist[i], OUTPUT);
    digitalWrite(pinlist[i], LOW);
  }

  // Ultrasonic sensor
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // Initialize PWM for motors
  for (int i = 0; i < 4; i++) {
    initPWM(&motors[i]);
  }

  // Servo setup
  myservo.attach(servoPin);
  myservo.write(90); // Start at center
  delay(1000);
}

void loop() {
  sendTriggerPulse();
  uint32_t duration = pulseInSTM(echo, HIGH);
  float distance = duration / 58.0;

  if (distance < 15.0) {
    stopAllMotors();
    delay(100);
    moveBackward();
    delay(200);
    stopAllMotors();
    delay(100);

    myservo.write(0);
    delay(400);
    sendTriggerPulse();
    float leftDist = pulseInSTM(echo, HIGH) / 58.0;

    myservo.write(180);
    delay(400);
    sendTriggerPulse();
    float rightDist = pulseInSTM(echo, HIGH) / 58.0;

    myservo.write(90);
    delay(400);

    if (leftDist > rightDist) {
      turnLeft();
    } else {
      turnRight();
    }
    delay(600);
  } else {
    moveForward();
  }

  // Update PWM for motors
  for (int i = 0; i < 4; i++) {
    updatePWM(&motors[i]);
  }
}