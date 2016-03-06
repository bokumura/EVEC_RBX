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
const int motorOn = 5;   		//C6
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
	uint8_t tempState = 0x00;
  	tempState |= (!digitalRead(manCartFwd)) << FWD_IN_OFFSET;
  	tempState |= (!digitalRead(manCartBack)) << BACK_IN_OFFSET;
  	tempState |= (!digitalRead(cartAtFront)) << FWD_STOP_IN;
  	tempState |= (!digitalRead(cartAtBack)) << BACK_STOP_IN;
  	tempState |= (!digitalRead(manLiftUp)) << UP_IN_OFFSET;
  	tempState |= (!digitalRead(manLiftDown)) << DOWN_IN_OFFSET;
  	tempState |= (!digitalRead(liftAtBottom)) << DOWN_STOP_IN;
  	tempState |= (!digitalRead(vanTireSw)) << VAN_TIRE_SW;
  	return tempState;
}
enum State { INIT, VAN_READY, RAMP_READY, ALL_READY, STOP, START_EXCHANGE, POSITION_PACK, RAISE_LIFT, LOWER_LIFT, ACTUATORS_OUT, ACTUATORS_IN, COMPLETE, CHECK_BATTERIES, MOVE_BAT_TO_CHARGER, INIT_CHARGERS, WAIT_FOR_VAN};
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

void loop() {
	while (autoState != STOP) {
   	//checkSerial();
    	switch (autoState) {
      	case INIT:
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
            error();
          }
				  /* Could have more cases for when things go wrong...*/
          else {
            while(1) { 
              Serial.println("Error in INIT!");
              Serial.print("CurrState = ");
              Serial.println(currState, HEX);
            }
            Serial1.write(ERROR_SIGNAL);
            error();
          }
        break;
    
      	case INIT_CHARGERS:
          Serial.println("INIT_CHARGERS: ");
          Serial.println("Going to CHECK_BATTERIES");
				  autoState = CHECK_BATTERIES;
      	break;

      	case RAMP_READY:
          Serial.println("RAMP_READY: ");
          digitalWrite(READY_PIN, HIGH);
				  // Should send signal to Van to let know that Ramp is ready...
          Serial.println("Writing to van that ramp is ready.");
				  Serial1.write(0x01);
          currState = checkInputs();
          checkSerial();
          Serial.println("RAMP_READY: Van still on ramp, waiting for driver to hit button.");
          while ((currState == 0xC4 || currState == 0xC8) && autoState == RAMP_READY) {
            currState = checkInputs();
            checkSerial();
          }

          if(currState == 0x44 || currState == 0x48) {
            autoState = WAIT_FOR_VAN;
            Serial.println("Van drove off ramp. Going to WAIT_FOR_VAN.");
            digitalWrite(READY_PIN, LOW);
            Serial1.write(0x05); //sending signal for van.
          }
				  /* NEED TO DETERMINE MORE LOGIC.... */
          //Should send signal to van to let know that 
         	/* else if (autoState != START_EXCHANGE) {
            	autoState = INIT;
          }*/

          else if(currState != 0xC4 && currState != 0xC8) {
            Serial.print("Error occurred. Inputs changed! CurrState = ");
            Serial.println(currState, HEX);
            Serial1.write(ERROR_SIGNAL);
            error();
          }
        break;

			  case START_EXCHANGE:
          Serial.println("START_EXCHANGE: ");
          digitalWrite(READY_PIN, LOW);
          chargersOff();
         	autoState = RAISE_LIFT;
        break;

      	case MOVE_BAT_TO_CHARGER:
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
           	error();
         	}
       	break;

      	case POSITION_PACK:
          Serial.println("POSITION_PACK: ");
          if(packInFrontCart) {
					  if(digitalRead(cartAtBack) == LOW) {	//At back, move Fwd
						  movFwd();
					  }
					  else if(digitalRead(cartAtFront) == HIGH) { //Not @ either ends
						  error();
					  }
            fwdChargerOn();
					  autoState = WAIT_FOR_VAN;
          }  
          else if(packInBackCart) {
					  if(digitalRead(cartAtFront) == LOW) {	//At front, move Rev
						  movRev();
					  }
					  else if(digitalRead(cartAtBack) == HIGH) { //Not @ either ends
						  error();
					  }
            backChargerOn();
					  autoState = WAIT_FOR_VAN;
          }
          else {
          	error();
         	}
       	break;

        case RAISE_LIFT:
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
       	break;
			
      	case ACTUATORS_OUT:
          Serial.println("ACTUATORS_OUT: ");
          Serial.println("send act. out");
          if (sendMessage(0x06) == 0x02) {
          	Serial.println("act going out");
           	while (Serial1.available() == 0) {
				  		//SHOULD CHECK TO MAKE SURE THAT NOTHING GOES WRONG
					  	// WHILE TAKING OUT ACTUATORS (LIFT DROPS, ACTUATORS 
						  // NOT GOING OUT, ETC...
					  }
            if (Serial1.read() == 0x06) {
          		autoState = LOWER_LIFT;
          		Serial.println("Act out");
           	}
           	else {
           	  autoState = ACTUATORS_OUT;
           	}
         }
       break;

      case LOWER_LIFT:
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
      break;

      case ACTUATORS_IN:
        Serial.println("ACTUATORS_IN: ");
        if (sendMessage(0x08) == 0x02) {
          Serial.println("act going in");
          while (Serial1.available() == 0);
          if (Serial1.read() == 0x08) {
            autoState = LOWER_LIFT;
          }
          else {
            autoState = ACTUATORS_IN;
          }
        }
      break;

      case COMPLETE:
        Serial.println("COMPLETE: ");
        // May not need to write complete since actuator_in signals that.
        if (sendMessage(0x09) == 0x02) {
          //Serial1.write(0x09);
          autoState = INIT;
          exchangeDone = true; // Not sure if this is needed...
          Serial1.flush();
        }
      break;

      case CHECK_BATTERIES:
        Serial.println("CHECK_BATTERIES: ");
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
			  	error();
		  	}
      break;
			
			case WAIT_FOR_VAN: 
        Serial.println("WAIT_FOR_VAN: ");
				currState = checkInputs();
        checkSerial();
				while(currState == 0x44 || currState == 0x48 && (autoState == WAIT_FOR_VAN)) {
					currState = checkInputs();
          checkSerial();
				}

        if(currState != 0xC8 && currState != 0xC4) {
          if(digitalRead(liftAtBottom) == HIGH) {
            Serial.println("LIFT UP!!");
            Serial1.write(ERROR_SIGNAL);
            error();
          }
          else {
            Serial.println("Something happened in WAIT_FOR_VAN. Not sure what.");
            Serial1.write(ERROR_SIGNAL);
            error();
          }
        }
				autoState = RAMP_READY;
		  break;

    	}
  	}
}

void movFwd() {
	digitalWrite(motorOn, HIGH);
  	delay(500);
  	digitalWrite(moveCartFwd, HIGH);
  	delay(500);
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
  	delay(500);
  	digitalWrite(moveCartBack, HIGH);
  	delay(500);
  	digitalWrite(moveCartBack, LOW);
  	while(digitalRead(cartAtBack) == HIGH);
  	digitalWrite(motorOn, LOW);
}

void raiseLift() {
	Serial.println("raise lift");
  	if (sendMessage(0x04) == 0x03) {
		Serial.println("going up");
  		Serial1.flush();
      digitalWrite(moveLiftDown, LOW);
    	digitalWrite(moveLiftUp, HIGH);
    	while (Serial1.available() == 0) {
			// SHOULD ADD LOGIC HERE IN CASE LIFT CRASHES OR INPUTS CHANGE
			// OR IF LIFT DOESN'T WORK AND LIFT_DOWN = T STILL...
		}
    	digitalWrite(moveLiftUp, LOW);
   	Serial.println("stop");
    	if (Serial1.read() != 0x05) {
     		Serial.println("keep going");
      	raiseLift();
    	}
  	}
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
//bool checkForBattery(bool frontStation){
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
    	digitalWrite(ERROR_PIN, HIGH);
      digitalWrite(READY_PIN, LOW);
    	Serial.println("ERROR");
    	Serial.println(currState, HEX);
      while(1) {
          
  		}
}
uint8_t sendMessage(uint8_t message) {
	bool response = false;
  	uint8_t packate = 0x00;
  	Serial1.write(message);
  	int startTime = millis();
  	while (!response) {
   	if (Serial1.available() > 0) {
      	packate = Serial1.read();
      	response = true;
      	Serial.println(packate);
    	}
  	}
  	return packate;
}


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
      	else {
        		Serial1.write(0x02);
      	}
    	}
      if (temp == ERROR_SIGNAL) {
        error();
      }
  	}
}

void StopISR() {
  Serial.println("Emergency Stop Button Pressed!");
  Serial1.write(ERROR_SIGNAL);
  error();
}
