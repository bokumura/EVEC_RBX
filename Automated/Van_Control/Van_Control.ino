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
const int ACK = 0x02;

/* If signal not received within this time limit, 
 *  signal is lost. ADDED 3/9 */
const int LOST_SIGNAL = 1000;

volatile unsigned long last_micros = 0;

//Offsets of signals in curState
const int MAN_ACTUATORS_ENGAGE_OFFSET = 0;
const int MAN_ACTUATORS_DISENGAGE_OFFSET = 1;
const int ACTUATORS_ENGAGED_OFFSET = 2;
const int ACTUATORS_DISENGAGED_OFFSET = 3;

//Threshholds for actuator placement
const int ACTUATORS_DISENGAGED_THRESHHOLD = 35;
const int ACTUATORS_ENGAGED_THRESHHOLD = 750;
const int ACTUATOR_RANGE = 10;

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
uint16_t manState = 0x0000;
bool rampReady = false;
bool actsIn = true;

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
    digitalWrite(READY_PIN, LOW);
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

    tempState |= ((analogRead(frontActuatorLocationPin) < (ACTUATORS_DISENGAGED_THRESHHOLD + ACTUATOR_RANGE))
    && (analogRead(rearActuatorLocationPin) < (ACTUATORS_DISENGAGED_THRESHHOLD + ACTUATOR_RANGE)))
    << ACTUATORS_ENGAGED_OFFSET;
    
    tempState |= ((analogRead(frontActuatorLocationPin) > (ACTUATORS_ENGAGED_THRESHHOLD - ACTUATOR_RANGE)) 
    && (analogRead(rearActuatorLocationPin) > (ACTUATORS_ENGAGED_THRESHHOLD - ACTUATOR_RANGE)))
    << ACTUATORS_DISENGAGED_OFFSET;
   return tempState;
}

uint8_t manInputs() {
  uint8_t tempState = 0x00;
  tempState |= (!digitalRead(manActuatorsEngage)) << MAN_ACTUATORS_ENGAGE_OFFSET;
  tempState |= (!digitalRead(manActuatorsDisengage)) << MAN_ACTUATORS_DISENGAGE_OFFSET;
  
  tempState |= ((analogRead(frontActuatorLocationPin) < ACTUATORS_DISENGAGED_THRESHHOLD)
    && (analogRead(rearActuatorLocationPin) < ACTUATORS_DISENGAGED_THRESHHOLD))
    << ACTUATORS_ENGAGED_OFFSET;
    
    tempState |= ((analogRead(frontActuatorLocationPin) > ACTUATORS_ENGAGED_THRESHHOLD) 
    && (analogRead(rearActuatorLocationPin) > ACTUATORS_ENGAGED_THRESHHOLD))
    << ACTUATORS_DISENGAGED_OFFSET;
  return tempState;
}

enum State { INIT, WAIT_FOR_DRIVER, VAN_READY, RAMP_READY, ALL_READY, STOP, START, EXCHANGE, RAISE_LIFT, WAIT, ACTUATORS_OUT, ACTUATORS_IN, COMPLETE};
//volatile State autoState = INIT;
State autoState = INIT;
void loop() {
  while(autoState != STOP) {
    //checkSerial();
      switch(autoState) {
        case INIT: {
          Serial.println("VAN_INIT: ");
          //delay(5000);
          currState = checkInputs();
          checkSerial();
          Serial.print("currState: ");
          Serial.println(currState, HEX);
          if(digitalRead(LIFT_UP_PIN) == LOW) {
            Serial.println("LIFT_UP! ERROR!");
            Serial1.write(ERROR_SIGNAL);
            error();
          }
          if(currState == 0x04) {   //Actuators engaged
            Serial.println("All good. Going to WAIT.");
            autoState = WAIT;
          }
          else {
            Serial.println("Actuators Not engaged. ERROR!");
            Serial1.write(ERROR_SIGNAL);
            error();
          }
      }
      break;
      case WAIT_FOR_DRIVER:
        EIFR = 0x01;
        Serial.println("VAN_WAIT_FOR_DRIVER: "); 
        while(autoState == WAIT_FOR_DRIVER) {
          checkSerial();
        }
        // Just want to wait for ISR to engage.
      break;

      case VAN_READY: {
        Serial.println("VAN_READY: ");
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
            Serial.println("VAN_START: ");
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
            Serial.println("VAN_EXCHANGE");
            digitalWrite(COMPLETE_LED_PIN, LOW);
            digitalWrite(READY_PIN, LOW);
            digitalWrite(EXCHANGE_PIN, HIGH);
            Serial.println("SET EXCHANGE PIN AND SET READY PIN LOW");
            currState = checkInputs();
            while(autoState == EXCHANGE && currState == 0x04) {
              checkSerial();
              currState = checkInputs();
            }
        }
        break;
      
        case RAISE_LIFT: {
          Serial.println("VAN_RAISE_LIFT: ");
          currState = checkInputs();
          if(actsIn) {
            if(currState != 0x04) {
              Serial.println("Actuators not in! Error!");
              Serial1.write(ERROR_SIGNAL);
              error();
            }
            else {
              actsIn = false;
            }
          }
          else {
            if(currState != 0x08) {
              Serial.println("Actuators not all the way out! Error");
              Serial1.write(ERROR_SIGNAL);
              error();
            }
            else {
              actsIn = true;
            }
          }
          Serial.println("raise lift");
          // Should I do checks here before sending the ok?
          if(digitalRead(LIFT_UP_PIN) == LOW) {
            Serial.println("LIFT Already up?! Error mode!");
            Serial1.write(ERROR_SIGNAL);
            error();
          }
          Serial1.write(0x03); 
          Serial.println("sent ok");
          while(digitalRead(LIFT_UP_PIN) == HIGH) {
            checkSerial();
          }
          Serial1.write(0x05);
          Serial.println("lift up");
          autoState = WAIT;
        }
        break;
      
        case WAIT: {
          Serial.println("VAN_WAIT: ");
          while(autoState == WAIT) {
            checkSerial(); 
          }
        }
        break;
      
        case ACTUATORS_OUT: {
            Serial.println("VAN_ACTUATORS_OUT: ");
            Serial.println("out");
            actuatorsOut();
            autoState = WAIT;
        }
        break;
      
        case ACTUATORS_IN: {
            Serial.println("VAN_ACTUATORS_IN: ");
            Serial.println("in");
            actuatorsIn();
            autoState = WAIT;
            //autoState = COMPLETE;
        }
        break;
      
        case COMPLETE: {
          Serial.println("VAN_COMPLETE: ");
          Serial1.write(0x02);
          digitalWrite(EXCHANGE_PIN,LOW);
          digitalWrite(COMPLETE_LED_PIN, HIGH);
          autoState = INIT;
          Serial1.flush();
        }
        break;
      }
    }
}

void manControl() {
  while(1) {
    manState = manInputs();
    Serial.print("VAN_MANUAL State: ");
    Serial.println(manState, HEX);
    currState = checkInputs();
    Serial.print("Currstates is: ");
    Serial.println(currState);
    switch(manState) {
      case 0x00: //Everything is off
      case 0x04: //Actuators engaged, not moving
      case 0x05: //Trying to engage actuators, but they are already engaged.
      case 0x08: //Actuators disengaged, not moving.
      case 0x0A: //Trying to disengage actuators, but they are already disengaged.
        stopActuators();
      break;

      case 0x01: //Engaging actuators, currently neither engaged nor disengaged.
      case 0x09: //Engaging actuators, currently disengaged.
        engageActuators();
      break;

      case 0x02: //Disengaging actuators, currently neither engaged nor disengaged.
      case 0x06: //Disengaging actuators, currently engaged.
        disengageActuators();
      break;

      default: //LOL GG DONE MESSED UP SON
        error();
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
    && (analogRead(rearActuatorLocationPin) > ACTUATORS_ENGAGED_THRESHHOLD))) {
      checkSerial();
    }
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
    digitalWrite(ERROR_PIN, HIGH);
    digitalWrite(COMPLETE_LED_PIN, LOW);
    digitalWrite(READY_PIN, LOW);
    digitalWrite(EXCHANGE_PIN, LOW);
    stopActuators();

    /* DEBUGGING! */
    while(1) {
      Serial.println("Van ERROR. Power down. ");  
      Serial.print("CurrState is: ");
      Serial.println(currState);
      Serial.print("AutoState is: ");
      Serial.println(autoState);
      Serial.print("front Actuator: ");
      int front = analogRead(frontActuatorLocationPin);
      Serial.println(front);
      Serial.print("rear Actuator: ");
      int back = analogRead(rearActuatorLocationPin);
      Serial.println(back);
      delay(5000);
    }
    manControl();
}

uint8_t sendMessage(uint8_t message) {
  bool response = false;
    uint8_t packate = 0x00;
    Serial1.write(message);
    int startTime = millis();
    /* Added 3/9 */
    int endTime = millis();
    while(!response) {
      if(Serial1.available() > 0) {
        response = true;
        packate = Serial1.read();
        Serial.println(packate); 
      }
      /* Added 3/9 */
      /* else if((endTime - startTime) > LOST_SIGNAL) {
        response = true;
        packate = ERROR_SIGNAL;
      }*/
      /* Added 3/9. Is this a good thing to add?? */
      /* else {
        Serial1.write(message);
      }*/
    }
    return packate;
}

void checkSerial() {
  uint8_t temp = 0x00;
    while(Serial1.available() > 0) {
    temp = Serial1.read();
    Serial.print("checkSerial: temp is: ");
    Serial.println(temp, HEX);
      if(temp == ERROR_SIGNAL) {
        /* DEBUGGING */
        Serial.println("Received error from ramp!");
        delay(3000);
        autoState = STOP;
        error();
      }
      if(temp == 0x01) {
        rampReady = true;
        Serial.println("Received ready from van. ");
        delay(1000);
        digitalWrite(READY_PIN, HIGH);
        Serial1.write(ACK);
        Serial.println("Received ready signal from van. Sending signal to ramp.");
        delay(1000);
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
          digitalWrite(COMPLETE_LED_PIN, LOW);
          digitalWrite(EXCHANGE_PIN, LOW);
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
    Serial.print("autoState:");
    Serial.println(autoState);
}


void StartISR() {
  //noInterrupts();
    if((long)(micros() - last_micros) > 15*1000) {
      if(autoState == WAIT_FOR_DRIVER) {
        autoState = VAN_READY;
        Serial.println("DRIVER_HIT_START_BUTTON! Go to VAN_READY.");
      }
      else {
        Serial.print("IN ISR! AutoState = ");
        Serial.println(autoState);
      }
     
    }
    Serial.println("IN ISR, but debounce fails!");
    last_micros = micros();
}
