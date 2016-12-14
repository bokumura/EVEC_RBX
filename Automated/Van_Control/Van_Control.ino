/*
INPUTS:
  Bit 0 = manActuatorsEngage
  Bit 1 = manActuatorsDisengage
  Bit 2 = actuatorsEngaged
  Bit 3 = actuatorsDisengaged
  Bit 4 = liftUp
  Bit 5 = NOT USED
  Bit 6 = NOT USED
  Bit 7 = NOT USED
OTHER INPUTS:
  driverStartButton [Interrupt]
  ignitionSwitchPin
*/ 

/*lift
OUTPUTS:
  1. movEngageActuators (actuatorsIn)
  2. movDisengageActuators (actuatorsOut)
OTHER OUTPUTS:
  ERROR_PIN
  READY_PIN
  EXCHANGE_PIN
  COMPLETE_PIN
*/

/* Serial Signals */
const int ERROR_SIGNAL = 0xFF; 
const int ACK = 0x02;

/* Constants */
const int MAN_ACTS_IN = 0x01;     // Signal for when manual Actuators Engaged
const int MAN_ACTS_OUT = 0x02;    // Signal for when manual Actuators Disengaged
const int ACTS_IN = 0x04;         // Signal for when ActsIn (engaged)
const int ACTS_OUT = 0x08;        // Signal for when ActsOut (disengaged)
const int LIFT_AT_TOP = 0x10;     // Signal for when liftAtTop
const int LIFT_NOT_UP = 0xEF;     // Mask to remove LIFT_AT_TOP from correctInput.  

const int LOST_SIGNAL = 1000;     //If signal not received within this time limit, signal was lost.
const int DEBOUNCE_TIME = 10;     //Amount of time input needs to remain steady for debounce input

volatile unsigned long last_micros;

//Offsets of signals in checkInputs/manInputs
const int MAN_ACTUATORS_ENGAGE_OFFSET = 0;
const int MAN_ACTUATORS_DISENGAGE_OFFSET = 1;
const int ACTUATORS_ENGAGED_OFFSET = 2;
const int ACTUATORS_DISENGAGED_OFFSET = 3;
const int LIFT_OFFSET = 4;

//Threshholds for actuator placement
const int ACTUATORS_DISENGAGED_THRESHHOLD = 35;
const int ACTUATORS_ENGAGED_THRESHHOLD = 750;
const int ACTUATOR_RANGE = 10;

//Inputs for van
const int manActuatorsEngage = 16;  //B2
const int manActuatorsDisengage = 15;  //B1
const int frontActuatorLocationPin = 0;  //F7
const int rearActuatorLocationPin = 1;  //F6
const int ignitionSwitchPin = 21; //F4
const int driverStartButton = 7; //E6 [Interrupt]
const int liftUp = 14;//B3

//Outputs for van
const int movActuatorsEngage = 6;  //D7
const int movActuatorsDisengage = 12;  //D6

//LED Outputs
const int ERROR_PIN = 22;  //F1
const int READY_PIN = 5; //C6
const int EXCHANGE_PIN = 13;//C7
const int COMPLETE_PIN = 23; //F0

//Current State
uint16_t currInput = 0x00;
uint16_t manState = 0x0000;
bool rampReady = false;
bool actsIn = true;
uint16_t correctInput = 0x00;

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    
    //Set actuator signals as inputs
    pinMode(manActuatorsEngage, INPUT);
    pinMode(manActuatorsDisengage, INPUT);
    pinMode(frontActuatorLocationPin, INPUT);
    pinMode(rearActuatorLocationPin, INPUT);
    
    //Set other inputs
    pinMode(ignitionSwitchPin, INPUT);
    pinMode(driverStartButton, INPUT);
    pinMode(liftUp, INPUT);
  
    //Set output pins
    pinMode(ERROR_PIN, OUTPUT);
    pinMode(READY_PIN, OUTPUT);
    pinMode(EXCHANGE_PIN, OUTPUT);
    pinMode(COMPLETE_PIN, OUTPUT);
    pinMode(movActuatorsEngage, OUTPUT);
    pinMode(movActuatorsDisengage, OUTPUT);
   
    //Set digital inputs as high (since active low)
    digitalWrite(manActuatorsEngage, HIGH);
    digitalWrite(manActuatorsDisengage, HIGH);
    digitalWrite(driverStartButton, HIGH);
    digitalWrite(liftUp, HIGH);
    
    //Start ignition pin as low
    digitalWrite(ignitionSwitchPin, LOW);
   
    //Initialize outputs to low
    digitalWrite(ERROR_PIN, LOW);
    digitalWrite(READY_PIN, LOW);
    digitalWrite(EXCHANGE_PIN, LOW);
    digitalWrite(COMPLETE_PIN, LOW);
    digitalWrite(movActuatorsEngage, LOW);
    digitalWrite(movActuatorsDisengage, LOW);
  
    attachInterrupt(digitalPinToInterrupt(driverStartButton), StartISR, FALLING);
}

/*This function takes all inputs and ORs them onto the proper
    position on the state to be returned (currInput). Digital inputs
    are active low, so they are inverted. Analog inputs are compared
    to their threshholds and set accordingly. 
    * Bit 0 = UNUSED (0) [Don't want to include manual inputs here.]
    * Bit 1 = UNUSED (1) [Don't want to include manual inputs here.]
    * Bit 2 = actuatorsEngaged (In)
    * Bit 3 = actuatorsDisengaged (Out)
    * Bit 4 = liftUp 
*/
uint8_t checkInputs() {
  uint8_t tempState = 0x00;
  tempState |= ((analogRead(frontActuatorLocationPin) < (ACTUATORS_DISENGAGED_THRESHHOLD + ACTUATOR_RANGE))
                 && (analogRead(rearActuatorLocationPin) < (ACTUATORS_DISENGAGED_THRESHHOLD + ACTUATOR_RANGE))) 
                 << ACTUATORS_ENGAGED_OFFSET;
  
  tempState |= ((analogRead(frontActuatorLocationPin) > (ACTUATORS_ENGAGED_THRESHHOLD - ACTUATOR_RANGE)) 
                 && (analogRead(rearActuatorLocationPin) > (ACTUATORS_ENGAGED_THRESHHOLD - ACTUATOR_RANGE))) 
                 << ACTUATORS_DISENGAGED_OFFSET;
    
  tempState |= ((!debouncePin(liftUp)) << LIFT_OFFSET);
  return tempState;
}

/* This function is similar to checkInputs, but also includes the manual
    inputs (i.e. pressing button to disengage/engage actuators). 
    * Bit 0 = manActuatorsEngage (In)
    * Bit 1 = manActuatorsDisengage (Out)
    * Bit 2 = actuatorsEngaged (In)
    * Bit 3 = actuatorsDisengaged (Out)
*/
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
  
  //tempState != ((!debouncePin(liftUp)) << LIFT_OFFSET);
  return tempState;
}


/* This function will debounce the input and ensure that the input
 * remains in a stable state for certain period of time.
`*/
int debouncePin(int pin) {
  int temp = digitalRead(pin);
  delay(DEBOUNCE_TIME);
  int temp2 = digitalRead(pin);
  if (temp2 == temp && temp == LOW)
    return LOW;
  else
    return HIGH;
}

enum ErrorState {NONE, LIFT_UP, LIFT_DOWN, ACTUATORS_DISENGAGED, ACTUATORS_ENGAGED, WRONG_INPUT, RAMP_ERROR, MISSED_SIGNAL};
ErrorState errState = NONE;

enum State {INIT, WAIT_FOR_VAN, WAIT_FOR_DRIVER, VAN_READY, WAIT_FOR_LIFT, WAIT_FOR_LIFT_UP, WAIT_FOR_ACTS, WAIT_FOR_ACTS_OUT, WAIT_FOR_ACTS_IN, WAIT_FOR_DONE,STOP};
State currState = INIT;
State prevState = INIT;

void loop() {
  while(currState != STOP) {
    printStateChange();
      switch(currState) {
        case INIT: {
          Serial.println("VAN_INIT: ");
          Serial.print("currInput: ");
          Serial.println(currInput, HEX);

          checkSerial();
          correctInput = ACTS_IN;
          checkCorrect();
          currState = WAIT_FOR_VAN;
        }
        break;

        case WAIT_FOR_VAN: {
          checkCorrect();
          checkSerial();
        }
        break;
        
        case WAIT_FOR_DRIVER: {
          // Just want to wait for ISR to engage OR for van to send signal (Error/not ready). 
          EIFR = 0x01; 
          Serial.println("VAN_WAIT_FOR_DRIVER: "); 
          checkCorrect();
          checkSerial();
        }
        break;

        case VAN_READY: {
          Serial.println("VAN_READY: ");
          checkSerial();
          checkIgnition();
          /* ignition was on. Jump back to switch statement b4 executing code here.*/
          if(currState != VAN_READY)
            break;
         
          checkCorrect();
          int message = sendMessage(0x01);
          if(message == ACK) {
            Serial.println("Started Exchange. Go and wait for lift signal");
            digitalWrite(COMPLETE_PIN, LOW);
            digitalWrite(READY_PIN, LOW);
            digitalWrite(EXCHANGE_PIN, HIGH);
            currState = WAIT_FOR_LIFT;
          }
          else {
            Serial.print("Message was: ");
            Serial.println(message, HEX);
            Serial1.write(ERROR_SIGNAL);
            errState = MISSED_SIGNAL;
            error();
          }
        }
        break;

        case WAIT_FOR_LIFT: {
          currInput = checkInputs();
          checkSerial();
          
          if(currInput != correctInput) {
            /* Check if lift has started to go down. */
            if(currInput == (correctInput & LIFT_NOT_UP)) {
              Serial.println("Just started putting lift down.");
              correctInput = currInput;
            }
            else {
              checkCorrect(); // Call this so can give correct errState. 
              /*Serial1.write(ERROR_SIGNAL);
              errState = WRONG_INPUT;
              error();
              */
            }
          }
        }
        break;

        case WAIT_FOR_LIFT_UP: {
          currInput = checkInputs();
          checkSerial();

          /* If lift is up and still correct input*/
          if(currInput == (correctInput | LIFT_AT_TOP)) {
            int message = sendMessage(0x05);
            if(message == ACK) {
              Serial.println("Lift up.");
              correctInput = (correctInput | LIFT_AT_TOP);
              currState = WAIT_FOR_ACTS;
            }
            else {
              Serial1.write(ERROR_SIGNAL);
              errState = MISSED_SIGNAL;
              error();
            }
          }

          else if(currInput != correctInput) {
            checkCorrect();     // Call this so can give correct errState. 
            /*Serial1.write(ERROR_SIGNAL);
            errState = WRONG_INPUT;
            error();
            */
          }
        }
        break;
        
        case WAIT_FOR_ACTS: {
          Serial.println("WAIT_FOR_ACTS: ");
          checkCorrect();
          checkSerial(); 
        }
        break;

        /* Can the inputs here be anything but acts in or acts out? */
        case WAIT_FOR_ACTS_OUT: {
          checkSerial();
          currInput = checkInputs();
          
          /* actuators are out */
          if(currInput == correctInput) {
            digitalWrite(movActuatorsDisengage, LOW); 
            if(sendMessage(0x06) != ACK) {
              Serial1.write(ERROR_SIGNAL);
              errState = MISSED_SIGNAL;
              error();
            }
            else {
              currState = WAIT_FOR_LIFT;
            }
          }
          else if(digitalRead(liftUp) == HIGH) {
            Serial.println("Lift not up! ERROR!");
            Serial1.write(ERROR_SIGNAL);
            errState = LIFT_DOWN;
            error();
          }
        }
        break;

        /* Can the inputs here be anything but acts in or acts out? */
        case WAIT_FOR_ACTS_IN: {
          checkSerial();
          currInput = checkInputs();
          /* actuators are in */
          if(currInput == correctInput) {
            digitalWrite(movActuatorsEngage, LOW);
            if(sendMessage(0x08) != ACK) {
              Serial1.write(ERROR_SIGNAL);
              errState = MISSED_SIGNAL;
              error();
            }
            else {
              currState = WAIT_FOR_DONE;
            }
          }
          else if(digitalRead(liftUp) == HIGH) {
            Serial.println("Lift not up! ERROR!");
            Serial1.write(ERROR_SIGNAL);
            errState = LIFT_DOWN;
            error();
          }
        }
        break;

        case WAIT_FOR_DONE: {
          if(currInput != correctInput) {
            /* Check if lift has started to go down... */
            if(currInput == (correctInput & LIFT_NOT_UP)) {
              Serial.println("Just started putting lift down.");
              correctInput = currInput;
            }
            else {
              checkCorrect();     // Call this so that can give correct errState.
              /*Serial1.write(ERROR_SIGNAL);
              errState = WRONG_INPUT;
              error();
              */
            }
          }
          checkSerial();
        }
        break;
      }
    }
}

void checkCorrect() {
  currInput = checkInputs();
  if(currInput != correctInput) {
    Serial1.write(ERROR_SIGNAL);
    /* correctInput has ACTS_IN and currInput does not... */
    if(((correctInput & ACTS_IN) == ACTS_IN) && ((currInput & ACTS_IN ) != ACTS_IN)) {
      errState = ACTUATORS_DISENGAGED;
    }
    /* Correct input has ACTS_OUT and currInput does not...*/
    else if(((correctInput & ACTS_OUT) == ACTS_OUT) && ((currInput & ACTS_OUT) != ACTS_OUT)) {
      errState = ACTUATORS_ENGAGED;
    }
    /* Correct input has LIFT_UP and currInput does not... */
    else if(((correctInput & LIFT_AT_TOP) == LIFT_AT_TOP) && ((currInput & LIFT_AT_TOP) != LIFT_AT_TOP)) {
      errState = LIFT_DOWN;
    }
    /* Correct input has LIFT_DOWN and currInput does not... */
    else if(((correctInput | LIFT_NOT_UP) == LIFT_NOT_UP) && ((currInput | LIFT_NOT_UP) != LIFT_NOT_UP)) {
      errState = LIFT_UP;
    }
    else {
      errState = WRONG_INPUT;
    }
    error();
  }
}

void manControl() {
  while(1) {
    manState = manInputs();
    Serial.print("VAN_MANUAL State: ");
    Serial.println(manState, HEX);
    switch(manState) {
      case 0x00: //Everything is off
      case ACTS_IN: //Actuators engaged, not requesting acts in or out
      case ACTS_IN | MAN_ACTS_IN: //Trying to engage actuators, but they are already engaged.
      case ACTS_OUT: //Actuators disengaged, not requesting acts in or out.
      case ACTS_OUT | MAN_ACTS_OUT: //Trying to disengage actuators, but they are already disengaged.
        stopActuators();
      break;

      case MAN_ACTS_IN: //Engaging actuators, currently neither engaged nor disengaged.
      case ACTS_OUT | MAN_ACTS_IN: //Engaging actuators, currently disengaged.
        engageActuators();
      break;

      case MAN_ACTS_OUT: //Disengaging actuators, currently neither engaged nor disengaged.
      case MAN_ACTS_OUT | ACTS_IN: //Disengaging actuators, currently engaged.
        disengageActuators();
      break;

      default: //LOL GG DONE MESSED UP SON
        error();
      break;
    }
  }
}

/* Print out if the state has changed. */
void printStateChange() {
   if(prevState != currState) {
      Serial.print("Changing state. Previous State = ");
      Serial.println(prevState);
       Serial.print("New State = ");
      Serial.println(currState);
    }
    prevState = currState;
}

/* May need to get rid of while loop? Or not since in the ISR?? */
void checkIgnition() {
  int count = 0;
  bool ignitionOn = false;
  while (count < 5 && !ignitionOn) {
    uint8_t  status = 0x00;
    delay(100);
    status |= digitalRead(ignitionSwitchPin);
    count++;
    if (status != 0x00)
      ignitionOn = true;
  }
  if (ignitionOn) {
    Serial.println("Ignition is on. Must turn off before pressing button.");
    currState = WAIT_FOR_DRIVER;
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

void error() {
  digitalWrite(COMPLETE_PIN, LOW);
  digitalWrite(READY_PIN, LOW);
  digitalWrite(EXCHANGE_PIN, LOW);
  stopActuators();

  /* DEBUGGING! */
  Serial.println("Van ERROR. Power down. ");  
  Serial.print("currInput is: ");
  Serial.println(currInput);
  Serial.print("currState is: ");
  Serial.println(currState);
  
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
    case LIFT_DOWN:
      for(int i = 0; i < 3; i++) {
          blinkError();
          delay(500);
          blinkError();
          delay(2000);
      }
    break;

    // Blink on/off 3 times
    case ACTUATORS_DISENGAGED:
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
    case ACTUATORS_ENGAGED:
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
    case WRONG_INPUT:
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
    
    // Blink on/off 7 times
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
    break;
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
    if(missCount > 8) {
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
    }
  }
  return packate; 
}

void checkSerial() {
  uint8_t temp = 0x00;
    if(Serial1.available() > 0) {
      temp = Serial1.read();
      Serial.print("checkSerial: temp is: ");
      Serial.println(temp, HEX);
      
      if(temp == ERROR_SIGNAL) {
        Serial.println("Received error from ramp!");
        errState = RAMP_ERROR;
        error();
      }
      
      if(temp == 0x01) {
        if(currState == WAIT_FOR_VAN) {
          rampReady = true;
          digitalWrite(READY_PIN, HIGH);
          Serial1.write(ACK);
          currState = WAIT_FOR_DRIVER;
        }
      }
      
      if(temp == 0x04) {
        // if we're already in WAIT_FOR_LIFT_UP, the ACK was missed, resend it
        if (currState == WAIT_FOR_LIFT_UP) {
          Serial1.write(ACK);
           currState = WAIT_FOR_LIFT_UP;
        }
        else if(currState == WAIT_FOR_LIFT) {
          currState = WAIT_FOR_LIFT_UP;
          Serial1.write(ACK); 
        }
        Serial.println("state to raise");
      }
      
      if(temp == 0x05) {
        if(currState == WAIT_FOR_DRIVER) { 
          Serial1.write(ACK);
          currState = WAIT_FOR_VAN;
          digitalWrite(READY_PIN, LOW);
          digitalWrite(COMPLETE_PIN, LOW);
          digitalWrite(EXCHANGE_PIN, LOW);
        }
        else if(currState == WAIT_FOR_VAN) {
          Serial1.write(ACK); // Ramp may have missed ACK from driving off ramp.
        }
      }
      
      if(temp == 0x06) {
        if(currState == WAIT_FOR_ACTS_OUT)
          Serial1.write(ACK);
        else if(currState == WAIT_FOR_ACTS) {
          currState = WAIT_FOR_ACTS_OUT; 
          /* Checking inputs for validity.. */
          Serial1.write(ACK);
          correctInput = ACTS_OUT | LIFT_AT_TOP; 
          disengageActuators();
        }
      }
      
      if(temp == 0x08) {
        if(currState == WAIT_FOR_ACTS_IN)
          Serial1.write(ACK);
        else if(currState == WAIT_FOR_ACTS) {
          currState = WAIT_FOR_ACTS_IN; 
          Serial1.write(ACK);
          correctInput = ACTS_IN | LIFT_AT_TOP;
          engageActuators();
        }
      }
      
      if(temp == 0x09) {
        if(currState == WAIT_FOR_DONE)
          Serial1.write(ACK);
          digitalWrite(EXCHANGE_PIN,LOW);
          digitalWrite(COMPLETE_PIN, HIGH);
          currState = INIT;
          Serial1.flush();
      }
      
    }
    Serial.print("currState:");
    Serial.println(currState);
}


void StartISR() {
  bool buttonPressed = true;
  for(int i = 0; i < 5 && buttonPressed; i++) {
    /* Delay 1/10 second and check if button is pressed still */
    delayMicroseconds(100000);
    if(digitalRead(driverStartButton) == HIGH) {  /* Button no longer pressed */
      buttonPressed = false;
    }
  }

  /* Button was pressed for .5 seconds. Should be safe to continue. */
  if(buttonPressed) {
    if(currState == WAIT_FOR_DRIVER) {
      currState = VAN_READY;
      Serial.println("DRIVER_HIT_START_BUTTON! Go to VAN_READY.");
    }
    else {
      Serial.print("IN ISR! AutoState = ");
      Serial.println(currState);
    }
  }
  else {
    Serial.println("button not pressed down long enough.");
  }
}

