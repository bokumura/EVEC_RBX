/*
INPUTS:
	Bit 0 = manActuatorsEngage
	Bit 1 = manActuatorsDisengage
	Bit 2 = actuatorsEngaged
	Bit 3 = actuatorsDisengaged

OTHER INPUTS:
	START_PIN
	LIFT_UP_PIN
*/ 

/*
OUTPUTS:
	1. movEngageActuators
  	2. movDisengageActuators

OTHER OUTPUTS:
	ERROR_PIN
	READY_PIN
	EXCHANGE_PIN
	COMPLETE_LED_PIN
*/

/* Serial signal for ERROR! */
const int ERROR_SIGNAL = 0xFF;

volatile unsigned long last_micros;
//Offsets of signals in curState
const int MAN_ACTUATORS_ENGAGE_OFFSET = 0;
const int MAN_ACTUATORS_DISENGAGE_OFFSET = 1;
const int ACTUATORS_DISENGAGED_OFFSET = 2;
const int ACTUATORS_ENGAGED_OFFSET = 3;

//Threshholds for actuator placement
const int ACTUATORS_DISENGAGED_THRESHHOLD = 35;
const int ACTUATORS_ENGAGED_THRESHHOLD = 750;

//Inputs for van
const int manActuatorsEngage = 16;  //B2 = Digital pin 16
const int manActuatorsDisengage = 15;  //B1 = Digital pin 15
const int frontActuatorLocationPin = 0;  //F7 = Analog pin 0
const int rearActuatorLocationPin = 1;  //F6 = Analog pin 1

//ERROR PIN
const int ERROR_PIN = 22;  //F1

//Complete Pin
const int COMPLETE_LED_PIN = 23; //F0

//Start Button
const int START_PIN = 7; //E6

//READY PIN
const int READY_PIN = 5; //C6

//Exchange PIN
const int EXCHANGE_PIN = 13;//C7

//Lift Up Pin
const int LIFT_UP_PIN = 14;//B3

//Outputs
const int movActuatorsEngage = 6;  //D7 = Digital pin 6
const int movActuatorsDisengage = 12;  //D6 = Digital pin 12



//Current State
uint16_t currState = 0x00;
bool rampReady = false;

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    
 	  //Set actuator signals as inputs
  	pinMode(manActuatorsEngage, INPUT);
  	pinMode(manActuatorsDisengage, INPUT);
  	pinMode(frontActuatorLocationPin, INPUT);
  	pinMode(rearActuatorLocationPin, INPUT);
  	pinMode(START_PIN, INPUT);
  	pinMode(LIFT_UP_PIN, INPUT);
  
  	//Set output pins
  	pinMode(ERROR_PIN, OUTPUT);
  	pinMode(READY_PIN, OUTPUT);
  	pinMode(EXCHANGE_PIN, OUTPUT);
  	pinMode(COMPLETE_LED_PIN, OUTPUT);
  	pinMode(movActuatorsEngage, OUTPUT);
  	pinMode(movActuatorsDisengage, OUTPUT);
 	 
  	//Set digital inputs as high (since active low)
  	digitalWrite(manActuatorsEngage, HIGH);
  	digitalWrite(manActuatorsDisengage, HIGH);
  	digitalWrite(START_PIN, HIGH);
  	digitalWrite(LIFT_UP_PIN, HIGH);
	 
  	//Initialize outputs to low
  	digitalWrite(ERROR_PIN, LOW);
  	digitalWrite(EXCHANGE_PIN, LOW);
	  digitalWrite(COMPLETE_LED_PIN, LOW);
  	digitalWrite(movActuatorsEngage, LOW);
  	digitalWrite(movActuatorsDisengage, LOW);
	
  	attachInterrupt(digitalPinToInterrupt(START_PIN), StartISR, FALLING);
}

/* This function takes all inputs and ORs them onto the proper
    position on the state to be returned (curstate). Digital inputs
    are active low, so they are inverted. Analog inputs are compared
    to their threshholds and set accordingly. */
uint8_t checkInputs() {
	uint8_t tempState = 0x00;
  	tempState |= (!digitalRead(manActuatorsEngage)) << MAN_ACTUATORS_ENGAGE_OFFSET;
  	tempState |= (!digitalRead(manActuatorsDisengage)) << MAN_ACTUATORS_DISENGAGE_OFFSET;
 	 
  	tempState |= ((analogRead(frontActuatorLocationPin) > ACTUATORS_ENGAGED_THRESHHOLD) 
    && (analogRead(rearActuatorLocationPin) > ACTUATORS_ENGAGED_THRESHHOLD))
    << ACTUATORS_ENGAGED_OFFSET;
  	tempState |= ((analogRead(frontActuatorLocationPin) < ACTUATORS_DISENGAGED_THRESHHOLD)
    && (analogRead(rearActuatorLocationPin) < ACTUATORS_DISENGAGED_THRESHHOLD))
    << ACTUATORS_DISENGAGED_OFFSET;
    
	  //Serial.print("Front Acctuator: ");
   //Serial.println(analogRead(frontActuatorLocationPin));
   //Serial.print("Back Acctuator: ");
   //Serial.println(analogRead(rearActuatorLocationPin));
   return tempState;
}
enum State { INIT, WAIT_FOR_DRIVER, VAN_READY, RAMP_READY, ALL_READY, STOP, START, EXCHANGE, RAISE_LIFT, WAIT, ACTUATORS_OUT, ACTUATORS_IN, COMPLETE};
volatile State autoState = INIT;
void loop() {
  while(autoState != STOP) {
    checkSerial();
    	switch(autoState) {
        case INIT: {
          currState = checkInputs();
        	Serial.println("TEMPState: ");
        	Serial.println(currState);
        	if(currState == 0x04) {		//Actuators engaged
            autoState = WAIT;
        	}
				  else {
            Serial1.write(ERROR_SIGNAL);
					  error();
				  }
      }
      break;
			case WAIT_FOR_DRIVER: 
        EIFR = 0x01;
        checkSerial();
				// Just want to wait for ISR to engage.
			break;

			case VAN_READY: {
				Serial.println("IN VAN_READY!");
				if(sendMessage(0x01) == 0x03) {
					autoState = EXCHANGE;
         Serial.println("SETTING AUTOSTATE TO EXCHANGE!");
				}
        else {
          Serial.println("DIDN'T SET AUTOSTATE TO EXCHANGE!");
        }
        
      }
      break;

      	case START: {
        		Serial.println(autoState);
        		digitalWrite(READY_PIN, LOW);
        		uint8_t response = sendMessage(0x01);
        		if(response == 0x03) {
         		autoState = EXCHANGE; 
        		}
        		else if(response == 0x02) {
          		autoState = VAN_READY;
        		}
      	}
      	break;
      
			case EXCHANGE: {
            Serial.println("IN EXCHANGE!!!");
				    digitalWrite(READY_PIN, LOW);
        		digitalWrite(EXCHANGE_PIN, HIGH);
            Serial.println("SET EXCHANGE PIN AND SET READY PIN LOW");
        		while(autoState == EXCHANGE) {
          		checkSerial(); 
        		}
      	}
      	break;
      
      	case RAISE_LIFT: {
       		Serial.println("raise lift");
       		Serial1.write(0x03); 
       		Serial.println("sent ok");
       		while(digitalRead(LIFT_UP_PIN) == HIGH);
       		Serial1.write(0x05);
       		Serial.println("lift up");
      	 	autoState = WAIT;
      	}
      	break;
      
      	case WAIT: {
       		checkSerial(); 
      	}
      	break;
      
      	case ACTUATORS_OUT: {
        		Serial.println("out");
        		actuatorsOut();
        		autoState = WAIT;
     	 	}
      	break;
      
      	case ACTUATORS_IN: {
        		Serial.println("in");
        		actuatorsIn();
        		autoState = WAIT;
      	}
      	break;
      
      	case COMPLETE: {
       		Serial1.write(0x02);
       		digitalWrite(EXCHANGE_PIN,LOW);
       		digitalWrite(COMPLETE_LED_PIN, HIGH);
       		delay(5000);
       		digitalWrite(COMPLETE_LED_PIN, LOW);
       		autoState = INIT;
       		Serial1.flush();
      	}
      	break;
    	}
  	}
}

void stopActuators() {
	digitalWrite(movActuatorsEngage, LOW);
  	digitalWrite(movActuatorsDisengage, LOW);
}

void engageActuators() {
	digitalWrite(movActuatorsEngage, HIGH);
  	digitalWrite(movActuatorsDisengage, LOW);
}

void disengageActuators() {
	digitalWrite(movActuatorsEngage, LOW);
  	digitalWrite(movActuatorsDisengage, HIGH);
}

void actuatorsOut() {
	Serial1.write(0x02);
  	digitalWrite(movActuatorsEngage, LOW);
  	digitalWrite(movActuatorsDisengage, HIGH);
  	while(!((analogRead(frontActuatorLocationPin) > ACTUATORS_ENGAGED_THRESHHOLD) 
    && (analogRead(rearActuatorLocationPin) > ACTUATORS_ENGAGED_THRESHHOLD)));
  	digitalWrite(movActuatorsDisengage, LOW); 
  	Serial1.write(0x06); 
}

void actuatorsIn() {
	  Serial1.write(0x02);
  	digitalWrite(movActuatorsDisengage, LOW);
  	digitalWrite(movActuatorsEngage, HIGH);
  	while(!((analogRead(frontActuatorLocationPin) < ACTUATORS_DISENGAGED_THRESHHOLD)
    && (analogRead(rearActuatorLocationPin) < ACTUATORS_DISENGAGED_THRESHHOLD))) {
      checkSerial();
    }
  	digitalWrite(movActuatorsEngage, LOW); 
  	Serial1.write(0x08); 
}

void error() {
	while (1) {
		Serial.println("Van ERROR. Power down. ");
		stopActuators();
		digitalWrite(ERROR_PIN, HIGH);
	}
}

uint8_t sendMessage(uint8_t message) {
	bool response = false;
  	uint8_t packate = 0x00;
 
  	Serial1.write(message);
  	int startTime = millis();
  	while(!response) {
   	if(Serial1.available() > 0) {
      	response = true;
      	packate = Serial1.read();
      	Serial.println(packate); 
    	}
  	}
  	return packate;
}

void checkSerial() {
	uint8_t temp = 0x00;
  	while(Serial1.available() > 0) {
		temp = Serial1.read();
	 	  if(temp == ERROR_SIGNAL) {
			  autoState = STOP;
		 	  error();
	 	  }
    	if(temp == 0x01) {
     		rampReady = true;
        digitalWrite(READY_PIN, HIGH);
			  autoState = WAIT_FOR_DRIVER;
    	}
    	if(temp == 0x04) {
     		autoState = RAISE_LIFT; 
     		Serial.println("state to raise");
    	}
      if(temp == 0x05) {
        if(autoState == WAIT_FOR_DRIVER) {
          autoState = WAIT;
          digitalWrite(READY_PIN, LOW);
        }
      }
    	if(temp == 0x06) {
     		autoState = ACTUATORS_OUT; 
    	}
    	if(temp == 0x08) {
     		autoState = ACTUATORS_IN; 
    	}
    	if(temp == 0x09) {
     		autoState = COMPLETE; 
    	}
  	}
}


void StartISR() {
	//noInterrupts();
  	if((long)(micros() - last_micros) > 15*1000) {
   	if(autoState == WAIT_FOR_DRIVER) {
      	autoState = VAN_READY;
    }
    	//Serial.println("ISR");
  	}
  	last_micros = micros();
}
