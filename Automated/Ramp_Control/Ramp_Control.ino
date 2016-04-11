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
 input: <man> <Obj> <Back/Front | up/down>
 output: <Obj> At <Where>
 */

/*
OUTPUTS:
  1. moveCart[FWD/Back]
   2. motorOn
   3. moveLiftUp
  4. moveLiftDown
*/

/* Serial signal for ERROR! */
const int ERROR_SIGNAL = 0xFF;

/* Acknowledgement signal */
const int ACK = 0x02;

const int LOST_SIGNAL = 1000;

const int FWD_IN_OFFSET = 0;
const int BACK_IN_OFFSET = 1;
const int FWD_STOP_IN = 2;
const int BACK_STOP_IN = 3;
const int UP_IN_OFFSET = 4;
const int DOWN_IN_OFFSET = 5;
const int DOWN_STOP_IN = 6;
const int VAN_TIRE_SW = 7;


//Emergency Stop Input
const int emergencyStop = 2; //D1

//4 inputs for ramp
const int manCartFwd = 16;   //B2
const int manCartBack = 14;  //B3
const int cartAtFront = 4;   //D4
const int cartAtBack = 10;   //B6

//3 inputs for lift
const int manLiftUp = 15;    //B1
const int manLiftDown = 17;  //B0
const int liftAtBottom = 9;  //B5

//ERROR PIN
//const int ERROR_PIN = 7;    //E6
const int ERROR_PIN = 7;    //D5

//INPUT for van switch
const int vanTireSw = 3;  //D0

//READY PIN
const int READY_PIN = 11;     //B7

//4 outputs
const int moveCartFwd = 13;   //C7
const int moveCartBack = 13;  //C7
const int motorOn = 5;      //C6
const int moveLiftUp = 6;     //D7
const int moveLiftDown = 12;  //D6

//outputs for checking battery location
const int rearChargerSelect = 21; //F4
const int frontChargerSelect = 22; //F1
const int energiseCharger = 20; //F5

//input for checking battery location
const int frontBatteryCheck = 19; //F6
const int backBatteryCheck = 18;//F7

const int BATTERY_VOLTAGE_MIN_DIFFERENCE = 2;

//enum for battery location
typedef enum {FRONT, BACK} batteryLocation;

/* variables for debouncing buttons/switches */
/* TIRE BUTTON DEBOUNCE VARIABLES */
int tire_counter = 0; // how many times we have seen new value
int tire_reading;    // current value read from the input pin
int tire_current_state = HIGH; // the debounced input value. (not pressed == HIGH).
long tire_time = 0;  // the last time the output pin was sampled.

/* MAN LIFT DOWN DEBOUNCE VARIABLES */
int man_lift_down_counter = 0; // how many times we have seen new value
int man_lift_down_reading;    // current value read from the input pin
int man_lift_down_current_state = HIGH; // the debounced input value. (not pressed == HIGH).
long man_lift_down_time = 0;  // the last time the output pin was sampled.

const int DEBOUNCE_COUNT = 10;   // number of millis/samples to consider before declaring a debounced in

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);

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
    pinMode(ERROR_PIN, OUTPUT);
    pinMode(moveCartFwd, OUTPUT);
    pinMode(moveCartBack, OUTPUT);
    pinMode(motorOn, OUTPUT);
    pinMode(moveLiftUp, OUTPUT);
    pinMode(moveLiftDown, OUTPUT);
    pinMode(READY_PIN, OUTPUT);

    // Outputs for checking battery location
    pinMode(rearChargerSelect, OUTPUT);
    pinMode(frontChargerSelect, OUTPUT);
    pinMode(energiseCharger, OUTPUT);

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
    digitalWrite(motorOn, LOW);
    digitalWrite(moveLiftUp, LOW);
    digitalWrite(moveLiftDown, LOW);
    digitalWrite(READY_PIN, LOW);

    digitalWrite(rearChargerSelect, LOW);
    digitalWrite(frontChargerSelect, LOW);
    attachInterrupt(digitalPinToInterrupt(emergencyStop), StopISR, FALLING);
}

/* This function takes all of the inputs and adds it into
  the state to be returned (currstate). All inputs are
   active low. */
uint16_t checkInputs() {
/*
  uint8_t tempState = 0x00;
    
  bool vanTire = debounceTireSwitch();
  
  tempState |= (!digitalRead(manCartFwd)) << FWD_IN_OFFSET;
  tempState |= (!digitalRead(manCartBack)) << BACK_IN_OFFSET;
  tempState |= (!digitalRead(cartAtFront)) << FWD_STOP_IN;
  tempState |= (!digitalRead(cartAtBack)) << BACK_STOP_IN;
  tempState |= (!digitalRead(manLiftUp)) << UP_IN_OFFSET;
  tempState |= (!digitalRead(manLiftDown)) << DOWN_IN_OFFSET;
  tempState |= (!digitalRead(liftAtBottom)) << DOWN_STOP_IN;
  tempState |= (!vanTire) << VAN_TIRE_SW;
  return tempState;
  */

  uint8_t tempState = 0x00;
  
  tempState |= (!debouncePin(manCartFwd)) << FWD_IN_OFFSET;
  tempState |= (!debouncePin(manCartBack)) << BACK_IN_OFFSET;
  tempState |= (!debouncePin(cartAtFront)) << FWD_STOP_IN;
  tempState |= (!debouncePin(cartAtBack)) << BACK_STOP_IN;
  tempState |= (!debouncePin(manLiftUp)) << UP_IN_OFFSET;
  tempState |= (!debouncePin(manLiftDown)) << DOWN_IN_OFFSET;
  tempState |= (!debouncePin(liftAtBottom)) << DOWN_STOP_IN;
  tempState |= (!debouncePin(vanTireSw)) << VAN_TIRE_SW;
  return tempState;
}

uint16_t manInputs(){
  uint8_t tempState = 0x00;
  //bool manLiftDown = debounceManLiftDown():

  tempState |= (!digitalRead(manCartFwd)) << FWD_IN_OFFSET;
  tempState |= (!digitalRead(manCartBack)) << BACK_IN_OFFSET;
  tempState |= (!digitalRead(cartAtFront)) << FWD_STOP_IN;
  tempState |= (!digitalRead(cartAtBack)) << BACK_STOP_IN;  
  tempState |= (!digitalRead(manLiftUp)) << UP_IN_OFFSET;
  //tempState |= (!manLiftDown) << DOWN_IN_OFFSET;
  tempState |= (!digitalRead(manLiftDown)) << DOWN_IN_OFFSET;
  tempState |= (!digitalRead(liftAtBottom)) << DOWN_STOP_IN;
  return tempState;
}

int debouncePin(int pin)
{
  int temp = digitalRead(pin);
  delay(10);
  int temp2 = digitalRead(pin);
  if (temp2 == temp && temp == LOW)
    return LOW;
  else
    return HIGH;
}

bool debounceTireSwitch() {
  if(millis() != tire_time) {
    tire_reading = digitalRead(vanTireSw);
    if(tire_reading == tire_current_state && tire_counter > 0) {
      tire_counter--;
    }
    if(tire_reading != tire_current_state) {
	    tire_counter++;
    }

    // If the input has shown the same value for long enough, let's switch it
    if(tire_counter >= DEBOUNCE_COUNT) {
      tire_counter = 0;
      tire_current_state = tire_reading;
    }
    tire_time = millis();
  }
  return tire_current_state; 
}

bool debounceManLiftDown() {
  if(millis() != man_lift_down_time) {
     man_lift_down_reading = digitalRead(manLiftDown);
	    if(man_lift_down_reading == man_lift_down_current_state && man_lift_down_counter > 0) {
	      man_lift_down_counter--;
	    }
	    if(man_lift_down_reading != man_lift_down_current_state) {
	      man_lift_down_counter++;
	    }
	
    // If the input has shown the same value for long enough, let's switch it
    if(man_lift_down_counter >= DEBOUNCE_COUNT) {
      man_lift_down_counter = 0;
      man_lift_down_current_state = man_lift_down_reading;
    }
    man_lift_down_time = millis();
 }
 return man_lift_down_current_state; 
}
enum ErrorState { NONE, EMERGENCY_BUTTON, LIFT_UP, LIFT_NOT_RISING, INIT_ERROR, READY_ERROR, PACK_NOT_AT_EITHER_SIDE, WAIT_FOR_VAN_ERROR, VAN_ERROR, MISSED_SIGNAL};
ErrorState errState = NONE;

enum State { INIT, VAN_READY, RAMP_READY, ALL_READY, STOP, START_EXCHANGE, POSITION_PACK, RAISE_LIFT, LOWER_LIFT, ACTUATORS_OUT, ACTUATORS_IN, COMPLETE, MOVE_BAT_TO_CHARGER, INIT_CHARGERS, WAIT_FOR_VAN};
State autoState = INIT;
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
uint16_t prevState = 0x0000;
uint16_t currState = 0x0000;
uint16_t manState = 0x0000;

void loop() {
  while (autoState != STOP) {
      switch (autoState) {
        case INIT: {
          actuatorPosition = true;
          exchangeDone = false;
          prevState = currState;
          currState = checkInputs();
          checkSerial();
          Serial.println("State: ");
          Serial.println(currState, HEX);

          /* Need to check if emergencyStop was pressed when 
          starting initialization. Would need to go into error mode... */
          if(digitalRead(emergencyStop) == LOW) {
            Serial.println("Emergency Stop Button Pressed. ERROR!");
            Serial1.write(ERROR_SIGNAL);
            // Set something here for what kind of error...
            errState = EMERGENCY_BUTTON;
            error();
          }
        
          if(currState == 0x44 || currState == 0x48) {
            autoState = INIT_CHARGERS;
            Serial.println("Van off ramp. Going to INIT_CHARGERS from INIT.");
          }
          
          /* Ramp is ready for exchange to begin */
          else if(currState == 0xC4 || currState == 0xC8) {
            if(!packInFrontCart && !packInBackCart) {
              autoState = INIT_CHARGERS;
              Serial.println("Van on Ramp. Going to INIT_CHARGERS from INIT.");
            }
            else {
              if(packInFrontCart) {
                  fwdChargerOn();
              }
              else if(packInBackCart) {
                  backChargerOn();          
              }
              autoState = RAMP_READY; // Or could do position carts here...
              Serial.println("Van on ramp and already positioned carts correctly. Going to RAMP_READY from INIT.");
            }
          }
          
          else if(digitalRead(liftAtBottom) == HIGH) {
            Serial.println("LIFT UP!!");
            Serial1.write(ERROR_SIGNAL);
            errState = LIFT_UP;
            error();
          }

          else if(currState == 0x02 || currState == 0x03) {
            Serial.println("Pack not in Back Cart or in Front Cart!!");
            Serial1.write(ERROR_SIGNAL);
            errState = PACK_NOT_AT_EITHER_SIDE;
            error();
          }
          
          /* Could have more cases for when things go wrong...*/
          else {
              Serial.println("Error in INIT!");
              Serial.print("CurrState = ");
              Serial.println(currState, HEX);
              Serial1.write(ERROR_SIGNAL);
              errState = INIT_ERROR;
              error();
          }
        }
        break;
        
        case RAMP_READY: {
          Serial.println("RAMP_READY: ");
          digitalWrite(READY_PIN, HIGH);
          Serial.println("Writing to van that ramp is ready.");

          /* Sending signal to van that ramp ready. */
          if(sendMessage(0x01) != ACK) {  
            Serial1.write(ERROR_SIGNAL);
            errState = MISSED_SIGNAL;
            error();
          }
          
          currState = checkInputs();
          checkSerial();
          Serial.println("RAMP_READY: Van still on ramp, waiting for driver to hit button.");

          /* Waiting until driver hits start button */
          while ((currState == 0xC4 || currState == 0xC8) && autoState == RAMP_READY) {
            currState = checkInputs();
            Serial.print("In While RAMP_READY loop... currState is: ");
            Serial.println(currState, HEX);
            checkSerial();
          } 
          
          Serial.print("autoState is: ");
          Serial.println(autoState, HEX);

          if(currState == 0x44 || currState == 0x48) {
            Serial.print("IN WAIT_FOR_VAN: currState is: ");
            Serial.println(currState,HEX);
            autoState = WAIT_FOR_VAN;
            Serial.println("Van drove off ramp. Going to WAIT_FOR_VAN.");
            digitalWrite(READY_PIN, LOW);

            /* Sending signal to van to go to WAIT for ready signal */
            if(sendMessage(0x05) != ACK) {
              Serial1.write(ERROR_SIGNAL);
              errState = MISSED_SIGNAL;
              error();
            }
          }
          
          else if(currState != 0xC4 && currState != 0xC8) {
            if(currState == 0xC0) {
              Serial.println("Carts neither forward or back.");
              Serial1.write(ERROR_SIGNAL);
              errState = PACK_NOT_AT_EITHER_SIDE;
              error();
            }
            else {
              Serial.print("Error occurred. Inputs changed! CurrState = ");
              Serial.println(currState, HEX);
              Serial1.write(ERROR_SIGNAL);
              errState = READY_ERROR;
              error();
            }
          }
        }
        break;

        case START_EXCHANGE: {
          Serial.println("START_EXCHANGE: ");
          digitalWrite(READY_PIN, LOW);
          //checkSerial();
          chargersOff();
          autoState = RAISE_LIFT;
        }
        break;

        case MOVE_BAT_TO_CHARGER: {
          Serial.println("MOVE_BAT_TO_CHARGER: ");
          if (packInFrontCart) {
            movRev();
            packInFrontCart = false;
            packInBackCart = true;
            autoState = RAISE_LIFT;
          }
          else if (packInBackCart) {
            movFwd();
            packInBackCart = false;
            packInFrontCart = true;
            autoState = RAISE_LIFT;
          }
          else {
            Serial.println("Pack not in Back Cart or in Front Cart!!");
            Serial1.write(ERROR_SIGNAL);
            errState = PACK_NOT_AT_EITHER_SIDE;
            error();
          }
        }
        break;

        case POSITION_PACK: {
          Serial.println("POSITION_PACK: ");
          if(packInFrontCart) {
            if(digitalRead(cartAtBack) == LOW) {  //At back, move Fwd
              movFwd();
            }
            else if(digitalRead(cartAtFront) == HIGH) { //Not @ either ends
              Serial1.write(ERROR_SIGNAL);
              errState = PACK_NOT_AT_EITHER_SIDE;
              error();
            }
            fwdChargerOn();
            autoState = WAIT_FOR_VAN;
          }  
          else if(packInBackCart) {
            if(digitalRead(cartAtFront) == LOW) { //At front, move Rev
              movRev();
            }
            else if(digitalRead(cartAtBack) == HIGH) { //Not @ either ends
              Serial1.write(ERROR_SIGNAL);
              errState = PACK_NOT_AT_EITHER_SIDE;
              error();
            }
            backChargerOn();
            autoState = WAIT_FOR_VAN;
          }
          else {
            Serial1.write(ERROR_SIGNAL);
            errState = PACK_NOT_AT_EITHER_SIDE;
            error();
          }
        }
        break;

        case RAISE_LIFT: {
          Serial.println("RAISE_LIFT: ");
          raiseLift();
          if (actuatorPosition) {
            actuatorPosition = false;
            autoState = ACTUATORS_OUT;
          }
          else {
            actuatorPosition = true;
            autoState =  ACTUATORS_IN;
          }
        }
        break;
      
        case ACTUATORS_OUT: {
          Serial.println("ACTUATORS_OUT: ");
          Serial.println("send act. out");
          int message = sendMessage(0x06);
          if (message == ACK) {
            Serial.println("act going out");
            while (Serial1.available() == 0) {
              //SHOULD CHECK TO MAKE SURE THAT NOTHING GOES WRONG
              // WHILE TAKING OUT ACTUATORS (LIFT DROPS, ACTUATORS 
              // NOT GOING OUT, ETC...
              checkSerial();
            }
            if (Serial1.read() == 0x06) {
              autoState = LOWER_LIFT;
              Serial1.write(ACK);
              Serial.println("Act out");
            }
            else {
              Serial1.write(ERROR_SIGNAL);
              errState = MISSED_SIGNAL;
              error();
            }
         }
         else if(message == 0x05) {
          if(sendMessage(0x06) != ACK) {
            Serial1.write(ERROR_SIGNAL);
            errState = MISSED_SIGNAL;
            error();
          }
          else {
            while(Serial1.available() == 0);
            /* Actuators all the way in */
            if(Serial1.read() == 0x06) {
              Serial1.write(ACK);
              autoState = LOWER_LIFT;
              Serial1.write(ACK);
              Serial.println("Act out");
            }
            else {
              Serial1.write(ERROR_SIGNAL);
              errState = MISSED_SIGNAL;
              error();
            }
          }
         }
         else {
          Serial1.write(ERROR_SIGNAL);
          errState = MISSED_SIGNAL;
          error();
         }
       }
       break;

      case LOWER_LIFT: {
        Serial.println("LOWER_LIFT: ");
        lowerLift();
        if (!exchangeDone) {
          exchangeDone = true;
          autoState = MOVE_BAT_TO_CHARGER;
        }
        else {
          exchangeDone = false;
          autoState = COMPLETE;
        }
      }
      break;

      case ACTUATORS_IN: {
        Serial.println("ACTUATORS_IN: ");

        /* Requesting to put actuators in */
        int message = sendMessage(0x08); 
        if (message == ACK) {
          Serial.println("act going in");
          while (Serial1.available() == 0);
          /* Actuators all the way in */
          if (Serial1.read() == 0x08) {
            Serial1.write(ACK);
            autoState = LOWER_LIFT;
          }
          else {
            Serial1.write(ERROR_SIGNAL);
            errState = MISSED_SIGNAL;
            error();
          }
        }
        else if(message == 0x05) { 
          if(sendMessage(0x08) != ACK) {
            Serial1.write(ERROR_SIGNAL);
            errState = MISSED_SIGNAL;
            error();
          }
          else {
            while(Serial1.available() == 0);
            /* Actuators all the way in */
            if(Serial1.read() == 0x08) {
              Serial1.write(ACK);
              autoState = LOWER_LIFT;
            }
            else {
              Serial1.write(ERROR_SIGNAL);
              errState = MISSED_SIGNAL;
              error();
            }
          }
        }
        else {
          Serial1.write(ERROR_SIGNAL);
          errState = MISSED_SIGNAL;
          error();
        }
      }
      break;

      case COMPLETE: {
        Serial.println("COMPLETE: ");
        if (sendMessage(0x09) == ACK) {
          autoState = INIT;
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
        /*if (!frontChecked && (digitalRead(cartAtFront) == LOW)) { // Carts at Front
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
              error();
            }
            else if ((backBatteryVoltage <= BATTERY_VOLTAGE_MIN_DIFFERENCE) 
             && (frontBatteryVoltage <= BATTERY_VOLTAGE_MIN_DIFFERENCE)) {
              error();
            }
            else if (backBatteryVoltage > frontBatteryVoltage) {
              packInBackCart = true;
              packInFrontCart = false;
              autoState = POSITION_PACK;
            }
            else if (frontBatteryVoltage > backBatteryVoltage) {
              packInFrontCart = true;
              packInBackCart = false;
              autoState = POSITION_PACK;
            }
            else {
              Serial.println("BattCheckError");
              error();
            }
        }*/
        frontChecked = true;
        backChecked = true;
        if(digitalRead(cartAtFront) == LOW) {
          Serial.println("cart at front");
          packInFrontCart = true;
          packInBackCart = false;
          autoState = POSITION_PACK;
        }
        else if(digitalRead(cartAtBack) == LOW) {
          Serial.println("cart at back");
          packInBackCart = true;
          packInFrontCart = false;
          autoState = POSITION_PACK;
        }
        else {
          Serial.println("No end stop pushed. [OR NO BATTERIES] FAILURE MODE!");
          Serial1.write(ERROR_SIGNAL);
          errState = PACK_NOT_AT_EITHER_SIDE;
          error();
        }
      }
      break;
      
      case WAIT_FOR_VAN: {
        Serial.println("WAIT_FOR_VAN: ");
        currState = checkInputs();
        checkSerial();
        while((currState == 0x44 || currState == 0x48) && (autoState == WAIT_FOR_VAN)) {
          currState = checkInputs();
          checkSerial();
        }

        if(currState != 0xC8 && currState != 0xC4) {
          if(digitalRead(liftAtBottom) == HIGH) {
            Serial.println("LIFT UP!!");
            Serial1.write(ERROR_SIGNAL);
            errState = LIFT_UP;
            error();
          }
          else {
            Serial.println("Something happened in WAIT_FOR_VAN. Not sure what.");
            Serial1.write(ERROR_SIGNAL);
            errState = WAIT_FOR_VAN_ERROR;
            error();
          }
        }

        if(currState == 0xC8 || currState == 0xC4) {
          Serial.println("van on ramp now!.");
          Serial.print("currState is: ");
          Serial.println(currState, HEX);
          autoState = RAMP_READY;
        }
      }
      break;

      }
    } 
}

void manControl() {
   digitalWrite(motorOn, HIGH);
   while(1) {
    // Manual mode!!! 
    manState = manInputs();
    Serial.print("Manual State: ");
    Serial.println(manState, HEX);
    switch(manState) {
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

void movFwd() {
    digitalWrite(motorOn, HIGH);
    delay(5000);
    digitalWrite(moveCartFwd, HIGH);
    delay(1000);
    digitalWrite(moveCartFwd, LOW);
    while(digitalRead(cartAtFront) == HIGH);
    digitalWrite(motorOn, LOW);
}

void stopCarts() {
  digitalWrite(motorOn, LOW);
  digitalWrite(moveCartBack, LOW);
}

void movRev() {
    digitalWrite(motorOn, HIGH);
    delay(5000);
    digitalWrite(moveCartBack, HIGH);
    delay(1000);
    digitalWrite(moveCartBack, LOW);
    while(digitalRead(cartAtBack) == HIGH);
    digitalWrite(motorOn, LOW);
}

void raiseLift() {
  Serial.println("raise lift");
    int message = sendMessage(0x04);
    if (message == 0x03) {
      Serial.println("going up");
      Serial1.flush();
      digitalWrite(moveLiftDown, LOW);
      digitalWrite(moveLiftUp, HIGH);
      while (Serial1.available() == 0) {
      // SHOULD ADD LOGIC HERE IN CASE LIFT CRASHES OR INPUTS CHANGE
      // OR IF LIFT DOESN'T WORK AND LIFT_DOWN = T STILL...
        checkSerial();
      }
      digitalWrite(moveLiftUp, LOW);
      if(digitalRead(liftAtBottom) == LOW) {
        Serial1.write(ERROR_SIGNAL);
        errState = LIFT_NOT_RISING;
        error(); 
      }
      Serial.println("stop");
      if (Serial1.read() != 0x05) {
        Serial1.write(ERROR_SIGNAL);
        errState = MISSED_SIGNAL;
        error();
      }
      else {
        Serial1.write(ACK);
      }
    }
    else if(message == 0x01) {
      /* Van missed ACK for driver start button */
      autoState = START_EXCHANGE; /* Go back a state */
    }
    else {
      Serial.println("Didn't receive signal!");
      Serial1.write(ERROR_SIGNAL);
      errState = MISSED_SIGNAL;
      error();
    }
    /* Need an else statement for if signal timed out */
    Serial.println("done");
}

void lowerLift() {
    Serial.println("lower lift");
    digitalWrite(moveLiftUp, LOW);
    digitalWrite(moveLiftDown, HIGH);
    while (digitalRead(liftAtBottom) == HIGH);
    digitalWrite(moveLiftDown, LOW);
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
    digitalWrite(energiseCharger, HIGH);
}

void backChargerOn() {
  digitalWrite(rearChargerSelect, HIGH);
    digitalWrite(energiseCharger, HIGH);
}

void chargersOff() {
  digitalWrite(rearChargerSelect, LOW);
  digitalWrite(frontChargerSelect, LOW);
  digitalWrite(energiseCharger, LOW);
}
//bool checkForBattery(bool frontStation) {
//
//  bool returnValue = false;
//  pinMode(optoIsolator, INPUT);
//  digitalWrite(optoIsolator, HIGH);
//  if(frontStation) {
//    digitalWrite(rearChargerSelect, LOW);
//    digitalWrite(frontChargerSelect, HIGH);
//  }
//  else {
//    digitalWrite(frontChargerSelect, LOW);
//    digitalWrite(rearChargerSelect, HIGH);
//  }
//  delay(50);
//  Serial.println(digitalRead(optoIsolator));
//  if (digitalRead(optoIsolator) == LOW) {
//   returnValue = true;
//  }
//
//  digitalWrite(rearChargerSelect, LOW);
//  digitalWrite(frontChargerSelect, LOW);
//  digitalWrite(optoIsolator, LOW);
//
//  return returnValue;
//  }

void error() {
      Serial.println("ERROR: ");
      chargersOff();
      stopLift();
      stopCarts();
      digitalWrite(READY_PIN, LOW);
      Serial.println("ERROR");
      Serial.println(currState, HEX);
      //sendMessage(currState);
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
    case LIFT_NOT_RISING:
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
    case INIT_ERROR:
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
    case READY_ERROR:
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
    case PACK_NOT_AT_EITHER_SIDE:
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
        delay(500);
        blinkError();
        delay(500);
        blinkError();
        delay(2000);
      }
    break;
    
   // Blink on/off 8 times
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
    while(Serial1.available() > 0) {
      temp = Serial1.read();
      if (temp == 0x01) {
        vanReady = true;
        if (autoState == RAMP_READY) {
            Serial1.write(0x03);
            autoState = START_EXCHANGE;
        }
        /*
        else {
            Serial1.write(0x02);
        }*/
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

