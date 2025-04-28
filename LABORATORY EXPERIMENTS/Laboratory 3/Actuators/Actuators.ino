#include <Servo.h>

Servo myservo;

//karan stepper pins
int pin1 = PA4;
int pin2 = PA5;
int pin3 = PA6;
int pin4 = PA7;


int dc_motor_pin1 = PB1;
int dc_motor_pin2 = PB0;


int pinslist[] = {pin1, pin2, pin3, pin4};
int speed_switch_pin = 3;


//stepper controller 
void stepper(int ms_delay) {
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, LOW);
  digitalWrite(pin4, LOW);
  delay(ms_delay);

  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
  digitalWrite(pin3, LOW);
  digitalWrite(pin4, LOW);
  delay(ms_delay);

  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, HIGH);
  digitalWrite(pin4, LOW);
  delay(ms_delay);

  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, LOW);
  digitalWrite(pin4, HIGH);
  delay(ms_delay);
}


// mode 1 = max speed; mode 2 = medium; mode 3 = slow;
void dc_motor_driver(bool reverse, int speed_mode){

  switch(speed_mode){
    case 1:
      if(!reverse){
        digitalWrite(dc_motor_pin1, 0);
        analogWrite(dc_motor_pin2, 255);
      }else{
        digitalWrite(dc_motor_pin2, 0);
        analogWrite(dc_motor_pin1, 255);
      }
    break;
    case 2:
      if(!reverse){
          digitalWrite(dc_motor_pin1, 0);
          analogWrite(dc_motor_pin2, 125);
        }else{
          digitalWrite(dc_motor_pin2, 0);
          analogWrite(dc_motor_pin1, 125);
        }
    break;
    case 3:
      if(!reverse){
          digitalWrite(dc_motor_pin1, 0);
          analogWrite(dc_motor_pin2, 55);
        }else{
          digitalWrite(dc_motor_pin2, 0);
          analogWrite(dc_motor_pin1, 55);
        }
    break;
    default:
    break;

  }

}

void servo_turn(){

  myservo.write(0);
  delay(500);
  myservo.write(180);

}



void setup(){

  for(int i=0; i < sizeof(pinslist)/sizeof(pinslist[0]); i++){
    pinMode(pinslist[i], OUTPUT);
    digitalWrite(pinslist[i], 0);
  }
  pinMode(dc_motor_pin1, OUTPUT);
  pinMode(dc_motor_pin2, OUTPUT);
  myservo.attach(9);
  pinMode(speed_switch_pin, INPUT);


}

void loop(){

  if(digitalRead(speed_switch_pin)){

    stepper(300);
    dc_motor_driver(true, 1);
    servo_turn();
    
  }
  //delay(100);

}