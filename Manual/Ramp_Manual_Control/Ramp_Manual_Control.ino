/* MANUAL MODE FOR THE RAMP! */
/*
 Bit 0 = manCartFwd
 Bit 1 = manCartBack
 Bit 2 = cartAtFront
 Bit 3 = cartAtBack
 Bit 4 = manLiftUp
 Bit 5 = manLiftDown
 Bit 6 = liftAtBottom
 Bit 7 = unassigned
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
   1. moveCartFwd/moveCartBack
   2. motorOn
   3. moveLiftUp
   4. moveLiftDown
 */
/* Serial signal for Emergency_Stop_Error */
const int ERROR_SIGNAL = 0xFF;

const int FWD_IN_OFFSET = 0;
const int BACK_IN_OFFSET = 1;
const int FWD_STOP_IN = 2;
const int BACK_STOP_IN = 3;
const int UP_IN_OFFSET = 4;
const int DOWN_IN_OFFSET = 5;
const int DOWN_STOP_IN = 6;

//4 inputs for ramp
const int manCartFwd = 16;   //B2
const int manCartBack = 14;  //B3 
const int cartAtFront = 4;   //D4
const int cartAtBack = 10;   //B6  

//Emergency Stop Input
const int emergencyStop = 2; //D1

//3 inputs for lift
const int manLiftUp = 15;    //B1
const int manLiftDown = 17;  //B0
const int liftAtBottom = 9;  //B5  

//ERROR PIN
const int ERROR_PIN = 7;    //E6
 
//4 outputs
const int moveCartFwd = 13;   //C7
const int moveCartBack = 13;   //C7
const int motorOn = 5;   //C6
const int moveLiftUp = 6;     //D7
const int moveLiftDown = 12;  //D6

//outputs for checking battery location
const int rearChargerSelect = 21; //F4
const int frontChargerSelect = 22; //F1

//input for checking battery location
const int optoIsolator = 19; //F6

//enum for battery location
typedef enum {FRONT, BACK} batteryLocation;

/* Variables to hold the current and previous states */
uint16_t prevState = 0x0000;
uint16_t currState = 0x0000;

void setup() {
  	Serial.begin(9600);
 
	//Set ramp signals as inputs
  	pinMode(manCartFwd, INPUT);
  	pinMode(manCartBack, INPUT);
  	pinMode(cartAtFront, INPUT);
  	pinMode(cartAtBack, INPUT);
  	pinMode(emergencyStop, INPUT);
  
	//Set lift signals as inputs
  	pinMode(manLiftUp, INPUT);
  	pinMode(manLiftDown, INPUT);
  	pinMode(liftAtBottom, INPUT);

	// Set output pins:
  	pinMode(ERROR_PIN,OUTPUT);
  	pinMode(moveCartFwd,OUTPUT);
  	pinMode(moveCartBack,OUTPUT);
  	pinMode(motorOn,OUTPUT);
  	pinMode(moveLiftUp, OUTPUT);
  	pinMode(moveLiftDown, OUTPUT);
  
	// Outputs for checking battery location
  	pinMode(rearChargerSelect, OUTPUT);
  	pinMode(frontChargerSelect, OUTPUT);
  
	// Input for opto isolator
  	pinMode(optoIsolator, INPUT);

	// Set the inputs as high (since active low)
  	digitalWrite(manCartFwd, HIGH);
  	digitalWrite(manCartBack, HIGH);
  	digitalWrite(cartAtFront, HIGH);
  	digitalWrite(cartAtBack, HIGH);
  	digitalWrite(emergencyStop, HIGH);
  
  	digitalWrite(manLiftUp, HIGH);
  	digitalWrite(manLiftDown, HIGH);
  	digitalWrite(liftAtBottom, HIGH);
  
  	// don't set optoIsolator high [it is done in checkForBattery()]

	// Set outputs as low initially
  	digitalWrite(ERROR_PIN,LOW);
  	digitalWrite(moveCartFwd, LOW);
  	digitalWrite(moveLiftUp, LOW);
  	digitalWrite(moveLiftDown, LOW);
  
  	digitalWrite(rearChargerSelect, LOW);
  	digitalWrite(frontChargerSelect, LOW);

  	/* SET THIS ONE HIGH! */
  	digitalWrite(motorOn, HIGH);

	/* INTERRPUT! */
	attachInterrupt(digitalPinToInterrupt(emergencyStop), StopISR, FALLING);
}

/* This function takes all of the inputs and adds it into 
   the state to be returned (currstate). All inputs are 
   active low. */
uint16_t checkInputs(){
  uint8_t tempState = 0x00;
  tempState |= (!digitalRead(manCartFwd)) << FWD_IN_OFFSET;
  tempState |= (!digitalRead(manCartBack)) << BACK_IN_OFFSET;
  tempState |= (!digitalRead(cartAtFront)) << FWD_STOP_IN;
  tempState |= (!digitalRead(cartAtBack)) << BACK_STOP_IN;  
  tempState |= (!digitalRead(manLiftUp)) << UP_IN_OFFSET;
  tempState |= (!digitalRead(manLiftDown)) << DOWN_IN_OFFSET;
  tempState |= (!digitalRead(liftAtBottom)) << DOWN_STOP_IN;
  return tempState;
}

void loop() {
  prevState = currState;
  currState = checkInputs();
  Serial.print("State: ");
  Serial.println(currState, HEX);
  delay(100);
  if (Serial1.available() > 0){
    uint16_t incoming = Serial1.read() << 8;
    incoming |= Serial1.read();
    Serial.print("External State: ");
    Serial.println(incoming);
  }

  switch (currState) {

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

  case 0x40: //LIFT IS DOWN, NOT MOVING
  case 0x44: //FWD END ON, DOWN ON, NOT MOVING
  case 0x48: //BACK END ON, DOWN ON, NOT MOVING
    stopCarts();
    stopLift();
    break;

  case 0x41: //MOVE FWD SWITCH IS ON
  case 0x49: //MOVE FWD, BACK END IS ON
    movFwd();
    break;

  case 0x45: // MOVE FWD, BUT FWD END IS ON
  case 0x4A: // MOVE BACKWARD, BUT BACK END IS ON
    stopCarts();
    break;

  case 0x42: //MOVE BACKWARD SWITCH IS ON
  case 0x46: //MOVE BACKWARD, FWD END IS ON
    movRev();
    break;

  case 0x14:  //MANUAL RAISE LIFT IS ON, CARTS IN FRONT
  case 0x18:  //MANUAL RAISE LIFT IS ON, CARTS IN BACK
  case 0x54:  //MANUAL RAISE LIFT IS ON, LIFT IS AT BOTTOM, CARTS IN FRONT
  case 0x58:  //MANUAL RAISE LIFT IS ON, LIFT IS AT BOTTOM, CARTS IN BACK
    movUp();
	 while(digitalRead(manLiftUp) == LOW);
	 stopLift();
    break;
    
  case 0x24:  //MOVE DOWN SWITCH IS ON, LIFT IS NOT AT BOTTOM, CARTS IN FRONT
  case 0x28:  //MOVE DOWN SWITCH IS ON, LIFT IS NOT AT BOTTOM, CARTS IN BACK
    movDown();
	 while((digitalRead(manLiftDown) == LOW) && digitalRead(liftAtBottom) == HIGH);
	 stopLift();
    break;
    
  case 0x64:  //MOVE DOWN, BUT DOWN ENDSTOP IS ON, CARTS ARE IN FRONT
  case 0x68:  //MOVE DOWN, BUT DOWN ENDSTOP IS ON, CARTS IN BACK
    stopLift();
    break;

  default: //LOL GG DONE MESSED UP SON
    error();
    break;
  } 
}

void movFwd(){
  digitalWrite(moveCartFwd,HIGH);
  delay(500);
  digitalWrite(moveCartFwd,LOW);
}

void stopCarts(){
  digitalWrite(moveCartFwd,LOW);
}

void movRev(){
  digitalWrite(moveCartBack,HIGH); 
  delay(500);
  digitalWrite(moveCartBack, LOW); 
}

void movUp(){
  digitalWrite(moveLiftDown, LOW);
  digitalWrite(moveLiftUp, HIGH);
}

void stopLift(){
  digitalWrite(moveLiftDown, LOW);
  digitalWrite(moveLiftUp, LOW);
}

void movDown(){
  digitalWrite(moveLiftUp, LOW);
  digitalWrite(moveLiftDown, HIGH);
}

/*
unsigned char checkForBattery(batteryLocation location){
  unsigned char foundBattery = 0;
  digitalWrite(optoIsolator, HIGH);
  digitalWrite(RearChargerSelect, LOW);
  digitalWrite(FrontChargerSelect, HIGH);
  delay(50);
  if (!digitalRead(optoIsolator))
    foundBattery = 1;
  digitalWrite(FrontChargerSelect, LOW);
  digitalWrite(optoIsolator, LOW);
  return 0;
} */

void error() {
   Serial1.write(ERROR_SIGNAL);
	while (1){
    	digitalWrite(moveCartFwd,LOW);
    	digitalWrite(motorOn, LOW);
    	digitalWrite(moveLiftUp, LOW);
    	digitalWrite(moveLiftDown, LOW);
    	digitalWrite(ERROR_PIN,HIGH);
    	Serial.println("ERROR");
    	Serial.println(currState);
    	delay(5000);
  	}
}

void StopISR() {
	Serial.println("Emergency Stop Button Pressed!");
	error();
}
