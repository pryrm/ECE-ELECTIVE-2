//DC Motor Controller
const int pin1 = PB0;
const int pin2 = PA7;


void setup(){
	Serial.begin(115200);
	pinMode(pin1, OUTPUT);
	digitalWrite(pin1, 0);
	
	
}

void loop(){
	
	char x = Serial.read();
  
	
	switch(x){
		case 'q':
			analogWrite(pin2, 255);
      Serial.println(x);
		break;
		case 'w':
			analogWrite(pin2, 125);
      Serial.println(x);
		break;
		case 'e':
			analogWrite(pin2, 50);
      Serial.println(x);
		break;
		default:
		break;
	}
	
}