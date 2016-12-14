/*
 Bit 0 = manCartFwd
 Bit 1 = manCartBack
 Bit 2 = cartAtFront
 Bit 3 = cartAtBack
 Bit 4 = manLiftUp
 Bit 5 = manLiftDown
 Bit 6 = liftAtBottom
 Bit 7 = vanTireButtonPressed
 Bit 8 = unassigned
 Bit 9 = unassigned
 Bit 10 = unassigned
 Bit 11 = unassigned
 Bit 12 = unassigned
 Bit 13 = unassigned
 Bit 14 = unassigned
 Bit 15 = unassigned
 input: <man> <Obj> <Back/Front | up/down>V
 output: <Obj> At <Where>
 */

/*
OUTPUTS:
  1. moveCart[FWD/Back]
  2. motorOn
  3. moveLiftUp
  4. moveLiftDown
*/

/* Serial Signals */
const int ERROR_SIGNAL = 0xFF;
const int ACK = 0x02;

/* Constants */
const int CART_FWD = 0x04;        // Signal for when cartAtFront (front button pressed)
const int CART_BACK = 0x08;       // Signal for when cartAtBack  (back button pressed)
const int LIFT_DOWN = 0x40;       // Signal for if LiftAtBottom
const int VAN_ON = 0x80;          // Signal for if Van on ramp 
const int VAN_OFF_RAMP = 0x7F;    // MASK to remove VAN_ON from correctInput.
const int LIFT_NOT_DOWN = 0xBF;   // Mask to remove LIFT_AT_BOTTOM from correctInput.

const int LOST_SIGNAL = 1000;     // If signal not received within this time limit, signal was lost.
const int DEBOUNCE_TIME = 10;     // Amount of time input needs to remain steady for debounce input

// Offsets of signals in checkInputs/manInputs
const int MAN_FWD_OFFSET = 0;
const int MAN_BACK_OFFSET = 1;
const int FWD_STOP_OFFSET = 2;
const int BACK_STOP_OFFSET = 3;
const int MAN_LIFT_UP_OFFSET = 4;
const int MAN_LIFT_DOWN_OFFSET = 5;
const int LIFT_DOWN_OFFSET = 6;
const int VAN_TIRE_SW = 7;


// Emergency Stop Input
const int emergencyStop = 2; //D1

// 4 inputs for carts
const int manCartFwd = 16;   //B2
const int manCartBack = 14;  //B3
const int cartAtFront = 4;   //D4
const int cartAtBack = 10;   //B6

// 3 inputs for lift
const int manLiftUp = 15;    //B1
const int manLiftDown = 17;  //B0
const int liftAtBottom = 9;  //B5

// 1 input for van switch
const int vanTireSw = 3;  //D0

//4 outputs
const int moveCartFwd = 13;   //C7
const int moveCartBack = 13;  //C7
const int motorOn = 5;        //C6
const int moveLiftUp = 6;     //D7
const int moveLiftDown = 12;  //D6

// LED Outputs
const int ERROR_PIN = 7;    //D5
const int READY_PIN = 11;     //B7

//outputs for checking battery location
const int rearChargerSelect = 21; //F4
const int frontChargerSelect = 22; //F1
const int energizeCharger = 20; //F5

//input for checking battery location
const int frontBatteryCheck = 19; //F6
const int backBatteryCheck = 18;//F7

const int BATTERY_VOLTAGE_MIN_DIFFERENCE = 2;

//enum for battery location
typedef enum {FRONT, BACK} batteryLocation;

const int DEBOUNCE_COUNT = 10;   // number of millis/samples to consider before declaring a debounced in

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);

    // Set ramp signals as inputs
    pinMode(manCartFwd, INPUT);
    pinMode(manCartBack, INPUT);
    pinMode(cartAtFront, INPUT);
    pinMode(cartAtBack, INPUT);
    pinMode(vanTireSw, INPUT);
    pinMode(emergencyStop, INPUT);

    // Set lift signals as inputs
    pinMode(manLiftUp, INPUT);
    pinMode(manLiftDown, INPUT);
    pinMode(liftAtBottom, INPUT);

    // Set output pins:
    pinMode(moveCartFwd, OUTPUT);
    pinMode(moveCartBack, OUTPUT);
    pinMode(motorOn, OUTPUT);
    pinMode(moveLiftUp, OUTPUT);
    pinMode(moveLiftDown, OUTPUT);
    pinMode(READY_PIN, OUTPUT);
    pinMode(ERROR_PIN, OUTPUT);
    
    // Outputs for checking battery location
    pinMode(rearChargerSelect, OUTPUT);
    pinMode(frontChargerSelect, OUTPUT);
    pinMode(energizeCharger, OUTPUT);

    // Input for battery checks
    pinMode(frontBatteryCheck, INPUT);
    pinMode(backBatteryCheck, INPUT);

    // Set the inputs as high (since active low)
    digitalWrite(manCartFwd, HIGH);
    digitalWrite(manCartBack, HIGH);
    digitalWrite(cartAtFront, HIGH);
    digitalWrite(cartAtBack, HIGH);
    digitalWrite(vanTireSw, HIGH);
    digitalWrite(emergencyStop, HIGH);

    digitalWrite(manLiftUp, HIGH);
    digitalWrite(manLiftDown, HIGH);
    digitalWrite(liftAtBottom, HIGH);

    // don't set optoIsolator high [it is done in checkForBattery()]

    // Set outputs as low initially
    digitalWrite(ERROR_PIN, LOW);
    digitalWrite(moveCartFwd, LOW);
    digitalWrite(moveCartBack, LOW);
    digitalWrite(motorOn, HIGH);
    digitalWrite(moveLiftUp, LOW);
    digitalWrite(moveLiftDown, LOW);
    digitalWrite(READY_PIN, LOW);

    digitalWrite(rearChargerSelect, LOW);
    digitalWrite(frontChargerSelect, LOW);
    digitalWrite(energizeCharger, LOW);
    attachInterrupt(digitalPinToInterrupt(emergencyStop), StopISR, FALLING);
}

/* This function takes all of the inputs and adds it into
  the state to be returned (currInput). All inputs are
   active low. 
   * Bit 0 = UNUSED (0) [Don't want to include manual inputs here.]
   * Bit 1 = UNUSED (0) [Don't want to include manual inputs here.]
   * Bit 2 = cartAtFront 
   * Bit 3 = cartAtBack
   * Bit 4 = UNUSED (0) [Don't want to include manual inputs here.]
   * Bit 5 = UNUSED (0) [Don't want to include manual inputs here.]
   * Bit 6 = liftAtBottom
   * Bit 7 = vanTirePressed
*/
uint16_t checkInputs() {
  uint8_t tempState = 0x00;
  
  tempState |= (!debouncePin(cartAtFront)) << FWD_STOP_OFFSET;
  tempState |= (!debouncePin(cartAtBack)) << BACK_STOP_OFFSET;
  tempState |= (!debouncePin(liftAtBottom)) << LIFT_DOWN_OFFSET;
  tempState |= (!debouncePin(vanTireSw)) << VAN_TIRE_SW;
  return tempState;
}

/* This function is similar to checkInputs, but also include the manual
 *  inputs (i.e. pressing button to move carts, and buttons to move lift).
 *  Bit 0 = manCartFwd
 *  Bit 1 = manCartBack
 *  Bit 2 = cartAtFront
 *  Bit 3 = cartAtBack
 *  Bit 4 = manLiftUp
 *  Bit 5 = manLiftDown
 *  Bit 6 = liftAtBotom
 *  Bit 7 = vanTirePressed
`*/ 
uint16_t manInputs(){
  uint8_t tempState = 0x00;
  
  tempState |= (!digitalRead(manCartFwd)) << MAN_FWD_OFFSET;
  tempState |= (!digitalRead(manCartBack)) << MAN_BACK_OFFSET;
  tempState |= (!digitalRead(cartAtFront)) << FWD_STOP_OFFSET;
  tempState |= (!digitalRead(cartAtBack)) << BACK_STOP_OFFSET;  
  tempState |= (!digitalRead(manLiftUp)) << MAN_LIFT_UP_OFFSET;
  tempState |= (!digitalRead(manLiftDown)) << MAN_LIFT_DOWN_OFFSET;
  tempState |= (!digitalRead(liftAtBottom)) << LIFT_DOWN_OFFSET;
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

enum ErrorState { NONE, EMERGENCY_BUTTON, LIFT_UP, CARTS_TIMED_OUT, PACK_NOT_AT_EITHER_SIDE, TOO_MANY_BATTERY_PACKS, NO_BATTERY_PACKS, VAN_ERROR, WRONG_INPUT, MISSED_SIGNAL};
ErrorState errState = NONE;

enum State { INIT, RAMP_READY, STOP, RAISE_LIFT, LOWER_LIFT, ACTUATORS_OUT, ACTUATORS_IN, COMPLETE,INIT_CHARGERS, WAIT_FOR_VAN};
State currState = INIT;
State prevState = INIT;
bool actuatorPosition = true;
bool exchangeDone = false;
bool packInFrontCart = false;
bool packInBackCart = false;
bool frontChecked = false;
bool backChecked = false;
bool vanReady = false;
int frontBatteryVoltage = 0;
int backBatteryVoltage = 0;

/* Variables to hold the current and previous states */
uint16_t currInput = 0x0000;
uint16_t manInput = 0x0000;
uint16_t correctInput = 0x00;

void loop() {
  while (currState != STOP) {
      printStateChange();
      switch (currState) {
        case INIT: {
          actuatorPosition = true;
          exchangeDone = false;
          currInput = checkInputs();
          checkSerial();

          /* Check if system started with emergency Stop pressed.*/
          if(digitalRead(emergencyStop) == LOW) {
            Serial.println("System started with Emergency Stop Button Pressed. ERROR!");
            Serial1.write(ERROR_SIGNAL);
            errState = EMERGENCY_BUTTON;
            error();
          }

          /* Van is not on ramp yet. */
          if(currInput == (CART_FWD | LIFT_DOWN) || currInput == (CART_BACK | LIFT_DOWN)) {
            currState = INIT_CHARGERS;
            correctInput = currInput;
            Serial.println("Van off ramp. Going to INIT_CHARGERS from INIT.");
          }
          
          /* Ramp is ready for exchange to begin */
          else if(currInput == (VAN_ON | CART_FWD | LIFT_DOWN)  || currInput == (VAN_ON | CART_BACK | LIFT_DOWN)) {
            /* First time running. Need to initialize ramp. */
            correctInput = currInput;
            if(!packInFrontCart && !packInBackCart) {
              currState = INIT_CHARGERS;
              Serial.println("Van on Ramp. Going to INIT_CHARGERS from INIT.");
            }
            /* Have already initialized system. Go to RAMP_READY. */
            else {
              if(packInFrontCart)
                  fwdChargerOn();
                  
              else if(packInBackCart)
                  backChargerOn();          
   
              /* Tell the van that ramp ready. */
              if(sendMessage(0x01) != ACK) {  
                Serial1.write(ERROR_SIGNAL);
                errState = MISSED_SIGNAL;
                error();
              }
              currState = RAMP_READY; 
              Serial.println("Van on ramp and already positioned carts correctly. Going to RAMP_READY from INIT.");
            }
          }

          /* Ramp in a non-valid position. Check if lift up... */
          else if(digitalRead(liftAtBottom) == HIGH) {
            Serial.println("LIFT UP!!");
            Serial1.write(ERROR_SIGNAL);
            errState = LIFT_UP;
            error();
          }

          /* Otherwise something else wrong. Possibly pack_not_at_either_side?? */
          else {
              Serial.println("Error in INIT!");
              Serial.print("currInput = ");
              Serial.println(currInput, HEX);
              Serial1.write(ERROR_SIGNAL);
              errState = WRONG_INPUT;
              error();
          }
        }
        break;
        
        case RAMP_READY: {
          digitalWrite(READY_PIN, HIGH);
          currInput = checkInputs();
          checkSerial();

          /* Check if van drove off ramp... */
          if(currInput == (correctInput & VAN_OFF_RAMP)) {
            currState = WAIT_FOR_VAN;
            correctInput = currInput;
            digitalWrite(READY_PIN, LOW);

            /* Sending signal to van letting know that van drove off ramp. */
            if(sendMessage(0x05) != ACK) {
              Serial1.write(ERROR_SIGNAL);
              errState = MISSED_SIGNAL;
              error();
            }
          }
          /* Make sure otherwise ramp in valid condition */
          else {
            checkCorrect();
          }
        }
        break;

        case RAISE_LIFT: {
          /* Wait until signal sent saying lift at top */
          checkSerial();
          currInput = checkInputs();
          
          /* Checking that either current inputs are correct inputs (w/ lift down) or correct inputs w/ lift up */
          if(currInput != correctInput && (currInput != (LIFT_DOWN | correctInput))) {
            Serial.print("Not correct inputs! CurrInput = ");
            Serial.println(currInput);
            Serial1.write(ERROR_SIGNAL);
            errState = WRONG_INPUT; 
            error();
          }
        }
        break;
      
        case ACTUATORS_OUT: {
          currInput = checkInputs();
          checkCorrect();
          checkSerial();
       }
       break;

      case LOWER_LIFT: {
        checkSerial();
        currInput = checkInputs();
        if(currInput != correctInput || currInput != (correctInput | LIFT_DOWN)) {
          Serial.print("Not correct inputs! CurrInput = ");
          Serial.println(currInput);
          Serial1.write(ERROR_SIGNAL);
          errState = WRONG_INPUT;
          error();
        }
        else if(currInput == (correctInput | LIFT_DOWN)) {
          stopLift();
          correctInput = currInput;
          if (!exchangeDone) {
            exchangeDone = true;
            moveToCharger();
            currState = RAISE_LIFT;
            raiseLift();
          }
          else {
            exchangeDone = false;
            currState = COMPLETE;
          }
        }
      }
      break;

      case ACTUATORS_IN: {
        currInput = checkInputs();
        checkCorrect();
        checkSerial();
      }
      break;

      case COMPLETE: {
        Serial.println("COMPLETE: ");
        if (sendMessage(0x09) == ACK) {
          currState = INIT;
          exchangeDone = true;
          Serial1.flush();
        }
        else {
          Serial1.write(ERROR_SIGNAL);
          errState = MISSED_SIGNAL;
          error();
        }
      }
      break;

      case INIT_CHARGERS: {
        Serial.println("INIT_CHARGERS: ");
        
        if (!frontChecked && (digitalRead(cartAtFront) == LOW)) { // Carts at Front
            Serial.println("Cart Forward");
            frontBatteryVoltage = analogRead(frontBatteryCheck);
            frontChecked = true;
            if (!backChecked) {
              movRev();
            }
          }
          else if (!backChecked && (digitalRead(cartAtBack) == LOW)) {
            Serial.println("cart back");
            backBatteryVoltage = analogRead(backBatteryCheck);
            backChecked = true;
            if (!frontChecked) {
              movFwd();
            }
          }
          else if (frontChecked && backChecked) {
            if ((backBatteryVoltage > BATTERY_VOLTAGE_MIN_DIFFERENCE) && 
             (frontBatteryVoltage > BATTERY_VOLTAGE_MIN_DIFFERENCE)) {
              Serial1.write(ERROR_SIGNAL);
              errState = TOO_MANY_BATTERY_PACKS;
              error();
            }
            else if ((backBatteryVoltage <= BATTERY_VOLTAGE_MIN_DIFFERENCE) 
             && (frontBatteryVoltage <= BATTERY_VOLTAGE_MIN_DIFFERENCE)) {
              Serial1.write(ERROR_SIGNAL);
              errState = NO_BATTERY_PACKS;
              error();
            }
            else if (backBatteryVoltage > frontBatteryVoltage) {
              packInBackCart = true;
              packInFrontCart = false;
              if(digitalRead(cartAtFront) == LOW) { //At front, move Rev
                movRev();
              }
              backChargerOn();
              correctInput = 0x06 | ((!digitalRead(vanTireSw)) << VAN_TIRE_SW);
              currState = WAIT_FOR_VAN;
            }
            else if (frontBatteryVoltage > backBatteryVoltage) {
              packInFrontCart = true;
              packInBackCart = false;
              if(digitalRead(cartAtBack) == LOW) {  //At back, move Fwd
                movFwd();
              }
              fwdChargerOn();
              correctInput = (CART_FWD | LIFT_DOWN) | ((!digitalRead(vanTireSw)) << VAN_TIRE_SW);
              currState = WAIT_FOR_VAN;
            }
            else {
              Serial.println("BattCheckError");
              Serial1.write(ERROR_SIGNAL);
              error();
            }
         }
      }
      break;
      
      case WAIT_FOR_VAN: {
        Serial.println("WAIT_FOR_VAN: ");
        currInput = checkInputs();
        checkSerial();
        
        // Check to see if it is valid input (off ramp) or on ramp. 
        if(currInput != correctInput && (currInput != (correctInput | VAN_ON))) { 
          Serial1.write(ERROR_SIGNAL);
          errState = WRONG_INPUT;
          error();
        }
        else if(currInput == (correctInput | VAN_ON)) {
          Serial.println("van on ramp now!.");
          correctInput = currInput;
          currState = RAMP_READY;

          /* Send signal to van saying Van_Ready. */
          if(sendMessage(0x01) != ACK) {  
            Serial1.write(ERROR_SIGNAL);
            errState = MISSED_SIGNAL;
            error();
          }
        }
      }
      break;

      }
    } 
}

void checkCorrect() {
  currInput = checkInputs();
  if(currInput != correctInput) {
    Serial1.write(ERROR_SIGNAL);
    errState = WRONG_INPUT;
    error();
  }
}

void manControl() {
   digitalWrite(motorOn, HIGH);
   while(1) {
    // Manual mode!!! 
    manInput = manInputs();
    Serial.print("Manual State: ");
    Serial.println(manInput, HEX);
    switch(manInput) {
      /* Lift not all the way down and not moving lift. */
      case 0x01:
      case 0x02:
      case 0x04:
      case 0x05:
      case 0x06:
      case 0x08:
      case 0x09:
      case 0x0A:
      break;

      case 0x40: //lift is down, not moving
      case 0x44: //fwd end on, down on, not moving
      case 0x48: //back end on, down on, not moving
        // Stop carts
        digitalWrite(moveCartBack, LOW);
        stopLift();
      break;

      case 0x41: //Move fwd switch is on
      case 0x49: //Move fwd, back end is on
        // Move cart fwd
        digitalWrite(moveCartFwd, HIGH);
        delay(500);
        digitalWrite(moveCartFwd, LOW);
      break;

      case 0x45: //move fwd, but fwd end is on
      case 0x4A: //move backward, but back end is on
        // Stop carts
        digitalWrite(moveCartBack, LOW);
      break;

      case 0x42: //move backward switch is on
      case 0x46: //move backward, fwd end is on
        // Moving cart back.
        digitalWrite(moveCartBack, HIGH);
        delay(500);
        digitalWrite(moveCartBack, LOW);
      break;

      case 0x14: //Manual raise lift is on, carts in front
      case 0x18: //manual raise lift is on, carts in back
      case 0x54: //Manual raise lift is on, lift is at bottom, carts in front.
      case 0x58: //Manual raise lift is on, lift is at bottom, carts in back.
        movUp();
        while(digitalRead(manLiftUp) == LOW);
        stopLift();
      break;

      case 0x24: //Move down switch is on, lift is not at bottom, carts in front.
      case 0x28: //Move down switch is on, lift is not at bottom, carts in back.
        movDown();
        while((digitalRead(manLiftDown) == LOW) && digitalRead(liftAtBottom) == HIGH);
        stopLift();
        break;

        case 0x64: //Move down, but down endstop is on, carts are in front.
        case 0x68: //Move down, but down endstop is on, carts in back
          stopLift();
        break;

        default: //LOL GG Done messed up
          //error();
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

void movFwd() {
  int count = 0;
  while(digitalRead(cartAtFront) == HIGH && count < 5) {
    if(digitalRead(cartAtBack) == LOW) {
      count++;
      moveCart();
    }
  }
  if(count >= 5) {
    Serial1.write(ERROR_SIGNAL);
    errState = CARTS_TIMED_OUT;
    error();
  }
}

void stopCarts() {
  digitalWrite(motorOn, LOW);
  digitalWrite(moveCartBack, LOW);
}

void movRev() {
    int count = 0;
    while(digitalRead(cartAtBack) == HIGH && count < 5) {
      if(digitalRead(cartAtFront) == LOW) {
        count++;
        moveCart();
      }
    }
    if(count >= 5) {
      Serial1.write(ERROR_SIGNAL);
      errState = CARTS_TIMED_OUT;
      error();
    }
}

void moveCart() {
  digitalWrite(moveCartFwd, HIGH);
  delay(1000);
  digitalWrite(moveCartFwd, LOW);
  delay(2000);
}

void signalExchange() {
  Serial.println("Start exchange. Turn off READY_PIN!");
  digitalWrite(READY_PIN, LOW);
  chargersOff();
}

void moveToCharger() {
  if (packInFrontCart) {
    movRev();
    packInFrontCart = false;
    packInBackCart = true;
    correctInput = CART_BACK | VAN_ON | LIFT_DOWN;
    
  }
  else if (packInBackCart) {
    movFwd();
    packInBackCart = false;
    packInFrontCart = true;
    correctInput = CART_FWD | VAN_ON | LIFT_DOWN;
  }
  else {
    Serial.println("Pack not in Back Cart or in Front Cart!!");
    Serial1.write(ERROR_SIGNAL);
    errState = PACK_NOT_AT_EITHER_SIDE;
    error();
  }
}

void actuatorsOut() {
  int message = sendMessage(0x06);
  if (message == ACK) {
    Serial.println("act going out");
    currState = ACTUATORS_OUT;
  }
  else {
    Serial1.write(ERROR_SIGNAL);
    errState = MISSED_SIGNAL;
    error();
  } 
}

void actuatorsIn() {
  if(sendMessage(0x08) != ACK) {
    Serial1.write(ERROR_SIGNAL);
    errState = MISSED_SIGNAL;
    error();
  }
}

void raiseLift() {
  Serial.println("raise lift");
  int message = sendMessage(0x04);
  if (message == ACK) {
    Serial.println("going up");
    Serial1.flush();
    digitalWrite(moveLiftDown, LOW);
    digitalWrite(moveLiftUp, HIGH);
  }

  else {
    Serial.println("Didn't receive signal!");
    Serial1.write(ERROR_SIGNAL);
    errState = MISSED_SIGNAL;
    error();
  }
}

void lowerLift() {
    Serial.println("lower lift");
    digitalWrite(moveLiftUp, LOW);
    digitalWrite(moveLiftDown, HIGH);
}

void movUp() {
  digitalWrite(moveLiftDown, LOW);
  digitalWrite(moveLiftUp, HIGH);
}

void stopLift() {
  digitalWrite(moveLiftDown, LOW);
  digitalWrite(moveLiftUp, LOW);
}

void movDown() {
  digitalWrite(moveLiftUp, LOW);
    digitalWrite(moveLiftDown, HIGH);
}

void fwdChargerOn() {
  digitalWrite(frontChargerSelect, HIGH);
    digitalWrite(energizeCharger, HIGH);
}

void backChargerOn() {
  digitalWrite(rearChargerSelect, HIGH);
    digitalWrite(energizeCharger, HIGH);
}

void chargersOff() {
  digitalWrite(rearChargerSelect, LOW);
  digitalWrite(frontChargerSelect, LOW);
  digitalWrite(energizeCharger, LOW);
}

void error() {
      Serial.println("ERROR: ");
      chargersOff();
      stopLift();
      stopCarts();
      digitalWrite(READY_PIN, LOW);
      Serial.println(currInput, HEX);
      printErrorMessage();
      digitalWrite(ERROR_PIN, HIGH);
      manControl();
}

/* Send a message to the van and recieve the message back. */
uint8_t sendMessage(uint8_t message) {
  int missCount = 0;

  bool response = false;
  uint8_t packate = 0x00;
  Serial1.write(message);
  unsigned long currTime = millis();
  while(!response) {
    if(missCount > 8){
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

/* Prints out the error message */
void printErrorMessage() {
  switch(errState) {

    // Blink on/off 1 time
    case EMERGENCY_BUTTON:
      for(int i = 0; i < 3; i++) {
        blinkError();
        delay(2000);
      }
    break;

    // Blink on/off 2 times
    case LIFT_UP:
      for(int i = 0; i < 3; i++) {
        blinkError();
        delay(500);
        blinkError();
        delay(2000);
      }
    break;

    // Blink on/off 3 times
    case CARTS_TIMED_OUT:
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
    case PACK_NOT_AT_EITHER_SIDE:
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
    case TOO_MANY_BATTERY_PACKS:
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
    case NO_BATTERY_PACKS:
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
    case VAN_ERROR:
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

     // Blink on/off 8 times
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
        delay(500);
        blinkError();
        delay(500);
        blinkError();
        delay(500);
        blinkError();
        delay(2000);
      }
    break;
    
    // Blink on/off 9 times
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
        delay(500);
        blinkError();
        delay(500);
        blinkError();
        delay(2000);
      }
    break;
      
    default:
      for(int i = 0; i < 3; i++) {
        digitalWrite(ERROR_PIN, HIGH);
        delay(1000);
        digitalWrite(ERROR_PIN, LOW);
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

/* Receives all of the signals from the van. */
void checkSerial() {
  uint8_t temp = 0x00;
    if(Serial1.available() > 0) {
      temp = Serial1.read();
      if(temp == 0x01) {
        vanReady = true;
        if (currState == RAMP_READY) {
            Serial1.write(ACK);
            /* Change LEDS to reflect starting exchange. */
            signalExchange();
            currState = RAISE_LIFT;
            /* Send signal and start raising lift*/
            correctInput = correctInput & LIFT_NOT_DOWN;
            raiseLift();
        }
      }
      if(temp == 0x05) {
        if(currState == RAISE_LIFT) {
          /* Got signal that lift at top. Stop lift! */
          Serial1.write(ACK);
          stopLift();
          if(actuatorPosition) {
            actuatorPosition = false;
            currState = ACTUATORS_OUT;
            /* Send signal to van to pull acts out */
            actuatorsOut();
            /* Want to have correctInput here that has lift up. */
          }
          else {
            actuatorPosition = true;
            currState =  ACTUATORS_IN;
            /* Send signal to van to put acts in */
            actuatorsIn();
          }
        }
      }
      if(temp == 0x06) {
        /* Van sent signal that ACTS out, and missed ACK signal. Send again. */
        if(currState == LOWER_LIFT) {
          Serial1.write(ACK);
        }
        else if(currState == ACTUATORS_OUT) {
          currState = LOWER_LIFT;
          Serial1.write(ACK);
          lowerLift();
        }
      }
      if(temp == 0x08) {
        /* Van sent signal that ACT in, and missed ACK signal. Send again. */
        if(currState == LOWER_LIFT) {
          Serial1.write(ACK);
        }
        else if(currState == ACTUATORS_IN) {
          currState = LOWER_LIFT;
          Serial1.write(ACK);
          lowerLift();
        }
      }
      if (temp == ERROR_SIGNAL) {
        errState = VAN_ERROR;
        error();
      }
    }
}

void StopISR() {
  Serial.println("Emergency Stop Button Pressed!");
  Serial1.write(ERROR_SIGNAL);
  errState = EMERGENCY_BUTTON;
  error();
}

