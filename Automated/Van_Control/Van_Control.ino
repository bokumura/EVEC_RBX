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

volatile unsigned long last_micros;

//Offsets of signals in curState
const int MAN_ACTUATORS_ENGAGE_OFFSET = 0;
const int MAN_ACTUATORS_DISENGAGE_OFFSET = 1;
const int ACTUATORS_ENGAGED_OFFSET = 2;
const int ACTUATORS_DISENGAGED_OFFSET = 3;

//Threshholds for actuator placement
const int ACTUATORS_DISENGAGED_THRESHHOLD = 35;
const int ACTUATORS_ENGAGED_THRESHHOLD = 750;
const int ACTUATOR_RANGE = 10;
const int ACTUATOR_RANGE_DISENGAGED = 100;

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

enum ErrorState {NONE, LIFT_UP, ACTUATORS_DISENGAGED, ACTUATORS_ENGAGED, READY_ERROR, WAIT_FOR_VAN_ERROR, RAMP_ERROR, MISSED_SIGNAL};
ErrorState errState = NONE;

enum State { INIT, WAIT_FOR_DRIVER, VAN_READY, STOP, EXCHANGE, RAISE_LIFT, WAIT, ACTUATORS_OUT, ACTUATORS_IN, COMPLETE};
//volatile State autoState = INIT;
State autoState = INIT;

void loop() {
  while(autoState != STOP) {
      switch(autoState) {
        case INIT: {
          Serial.println("VAN_INIT: ");
          currState = checkInputs();
          checkSerial();
          Serial.print("currState: ");
          Serial.println(currState, HEX);
          if(digitalRead(LIFT_UP_PIN) == LOW) {
            Serial.println("LIFT_UP! ERROR!");
            Serial1.write(ERROR_SIGNAL);
            errState = LIFT_UP;
            error();
          }
          if(currState == 0x04) {   //Actuators engaged
            Serial.println("All good. Going to WAIT.");
            autoState = WAIT;
          }
          else {
            Serial.println("Actuators Not engaged. ERROR!");
            Serial1.write(ERROR_SIGNAL);
            errState = ACTUATORS_DISENGAGED;
            error();
          }
        }
        break;
        case WAIT_FOR_DRIVER: {
          EIFR = 0x01;
          Serial.println("VAN_WAIT_FOR_DRIVER: "); 
          while(autoState == WAIT_FOR_DRIVER) {
            checkSerial();
          }
          // Just want to wait for ISR to engage.
        }
        break;

        case VAN_READY: {
          Serial.println("VAN_READY: ");
          int message =  sendMessage(0x01);
          if(message == 0x03) {
            autoState = EXCHANGE;
            Serial.println("SETTING AUTOSTATE TO EXCHANGE!");
          }
          else if(message == 0x04) {
            /* Must have missed 0x03 (ACK) and ramp in raiseLift */
            autoState = EXCHANGE; /* Just continue to go forward */
          }
          /* Possibly missed signal? Error? */
          else {
            Serial.println("DIDN'T SET AUTOSTATE TO EXCHANGE!");
            Serial1.write(ERROR_SIGNAL);
            errState = MISSED_SIGNAL;
            error();
            /* SHOULD I SET ERRSTATE HERE?? */
          }
        }
        break;

      /*
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
      */
      
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
              errState = ACTUATORS_DISENGAGED;
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
              errState = ACTUATORS_ENGAGED;
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
            errState = LIFT_UP;
            error();
          }
          /* Sending acknowledge to ramp */
          Serial1.write(0x03); 
          Serial.println("sent ok");
          while(digitalRead(LIFT_UP_PIN) == HIGH) {
            checkSerial();
          }
          int message = sendMessage(0x05); //0x06
          if(message == ACK) {
            Serial.println("lift up");
            autoState = WAIT;
          }
          else if(message == 0x08) {
            Serial.println("Missed ACK, but ramp received message. Put acts in. ");
            autoState = WAIT;
          }
          else if(message == 0x06) {
            Serial.println("Missed ACK, but ramp received message. Take acts out. ");
            autoState = WAIT;
          }
          else {
            Serial1.write(ERROR_SIGNAL);
            errState = MISSED_SIGNAL;
            error();
          }
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
          Serial1.write(ACK);
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
  Serial1.write(ACK);
  digitalWrite(movActuatorsEngage, LOW);
  digitalWrite(movActuatorsDisengage, HIGH);
  /* Do I need to add threshold here? */
  while(!((analogRead(frontActuatorLocationPin) > ACTUATORS_ENGAGED_THRESHHOLD) 
  && (analogRead(rearActuatorLocationPin) > ACTUATORS_ENGAGED_THRESHHOLD))) {
    checkSerial();
  }
  digitalWrite(movActuatorsDisengage, LOW); 
  if(sendMessage(0x06) != ACK) {
    Serial1.write(ERROR_SIGNAL);
    errState = MISSED_SIGNAL;
    error();
  }
}

void actuatorsIn() { 
  Serial1.write(ACK); 
  digitalWrite(movActuatorsDisengage, LOW);
  digitalWrite(movActuatorsEngage, HIGH);
  /* Do I need to add range to this? */
  while(!((analogRead(frontActuatorLocationPin) < ACTUATORS_DISENGAGED_THRESHHOLD)
  && (analogRead(rearActuatorLocationPin) < ACTUATORS_DISENGAGED_THRESHHOLD))) {
    checkSerial();
  }
  digitalWrite(movActuatorsEngage, LOW); 
  if(sendMessage(0x08) != ACK) {
    Serial1.write(ERROR_SIGNAL);
    errState = MISSED_SIGNAL;
    error();
  }
}

void error() {
  digitalWrite(COMPLETE_LED_PIN, LOW);
  digitalWrite(READY_PIN, LOW);
  digitalWrite(EXCHANGE_PIN, LOW);
  stopActuators();

  /* DEBUGGING! */
  Serial.println("Van ERROR. Power down. ");  
  Serial.print("CurrState is: ");
  Serial.println(currState);
  Serial.print("AutoState is: ");
  Serial.println(autoState);

  int front = analogRead(frontActuatorLocationPin);
  int back = analogRead(rearActuatorLocationPin);
  Serial.print("Front actuator pin is: ");
  Serial.println(front);
  Serial.print("Read actuator pin is: ");
  Serial.println(back);

  /*while(Serial1.available() == 0);
  int message = Serial1.read();
  Serial1.write(ACK);
  Serial.print("Ramp currState is: ");
  Serial.println(message);
  */
  delay(5000);
  printErrorMessage();
  digitalWrite(ERROR_PIN, HIGH);
  manControl();
}

void printErrorMessage() {
  
  switch(errState) {

    // Blink on/off 1 time
    case LIFT_UP:
      for(int i = 0; i < 3; i++) {
        blinkError();
        delay(2000);
      }
    break;

    // Blink on/off 2 times
    case ACTUATORS_DISENGAGED:
      for(int i = 0; i < 3; i++) {
          blinkError();
          delay(500);
          blinkError();
          delay(2000);
      }
    break;

    // Blink on/off 3 times
    case ACTUATORS_ENGAGED:
      for(int i = 0; i < 3; i++) {
          blinkError();
          delay(500);
          blinkError();
          delay(500);
          blinkError();
          delay(2000);
      }
    break;

    // Blink on/off 4 times
    case READY_ERROR:
      for(int i = 0; i < 3; i++) {
          blinkError();
          delay(500);
          blinkError();
          delay(500);
          blinkError();
          delay(500);
          blinkError();
          delay(2000);
      }
    break;

    // Blink on/off 5 times
    case WAIT_FOR_VAN_ERROR:
      for(int i = 0; i < 3; i++) {
          blinkError();
          delay(500);
          blinkError();
          delay(500);
          blinkError();
          delay(500);
          blinkError();
          delay(500);
          blinkError();
          delay(2000);
      }
    break;

    // Blink on/off 6 times
    case RAMP_ERROR:
      for(int i = 0; i < 3; i++) {
          blinkError();
          delay(500);
          blinkError();
          delay(500);
          blinkError();
          delay(500);
          blinkError();
          delay(500);
          blinkError();
          delay(500);
          blinkError();
          delay(2000);
      }
    break;

    case MISSED_SIGNAL:
      for(int i = 0; i < 3; i++) {
          blinkError();
          delay(500);
          blinkError();
          delay(500);
          blinkError();
          delay(500);
          blinkError();
          delay(500);
          blinkError();
          delay(500);
          blinkError();
          delay(500);
          blinkError();
          delay(2000);
      }
  }
}

/* Will blink the error light once */
void blinkError() {
  digitalWrite(ERROR_PIN, HIGH);
  delay(500);
  digitalWrite(ERROR_PIN, LOW);
}

uint8_t sendMessage(uint8_t message) {
  int missCount = 0;

  bool response = false;
  uint8_t packate = 0x00;
  Serial1.write(message);
  unsigned long currTime = millis();
  while(!response) {
    if(missCount > 5){
      return ERROR_SIGNAL;
    }
    if(Serial1.available() > 0) {
      response = true;
      packate = Serial1.read();
      Serial.println(packate); 
    }
    if((millis() - currTime) > LOST_SIGNAL) {
      /*RESEND*/
      Serial1.write(message);
      missCount++;
      currTime = millis();
      //response = true;
      //packate = ERROR_SIGNAL;
    }
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
        //autoState = STOP;
        errState = RAMP_ERROR;
        error();
      }
      if(temp == 0x01) {
        rampReady = true;
        digitalWrite(READY_PIN, HIGH);
        Serial1.write(ACK);
        autoState = WAIT_FOR_DRIVER;
      }
      if(temp == 0x04) {
        // if we're already in RAISE_LIFT, the ACK was missed, resend it
        if (autoState == RAISE_LIFT)
          Serial1.write(0x03);
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
        if(autoState == ACTUATORS_OUT)
          Serial1.write(ACK);
        autoState = ACTUATORS_OUT; 
      }
      if(temp == 0x08) {
        if(autoState == ACTUATORS_IN)
          Serial1.write(ACK);
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
