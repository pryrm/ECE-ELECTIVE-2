//Speed Adjustment
#include <HCSR04.h>

const int trig = PB12;
const int echo = PB13;
const int pin1 = PA5;
const int pin2 = PA6;


HCSR04 hc(trig, echo); 


void setup(){
  pinMode(pin1, OUTPUT);
  digitalWrite(pin1 , 0);

}

void loop(){
   float dist = hc.dist(); 

   if(dist < 20){
    //distance and speed condition 1
    analogWrite(pin2, 255);
   }else if(dist > 20 && dist < 60){
    //distance and speed condition 2
    analogWrite(pin2, 120);
   }else{
    //distance and speed condition 3
    analogWrite(pin2, 55);
   }


    delay(60);                 
}