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
   1. moveCartFwd
   2. moveCartBack
   3. moveLiftUp
   4. moveLiftDown
 */
const int FWD_IN_OFFSET = 0;
const int BACK_IN_OFFSET = 1;
const int FWD_STOP_IN = 2;
const int BACK_STOP_IN = 3;
const int UP_IN_OFFSET = 4;
const int DOWN_IN_OFFSET = 5;
const int DOWN_STOP_IN = 6;

//4 inputs for ramp
const int fwdIn = 16;      //B2 manCartFwd
const int backIn = 14;     //B3 manCartBack
const int fwdStopIn = 4;   //D4 cartAtFront
const int backStopIn = 10; //B6  cartAtBack

//3 inputs for lift
//const int manualUp = 6;     //D7  manLiftUp
const int manualUp = 15;     //B1  manLiftUp
//const int manualDown = 12;  //D6  manLiftDown
const int manualDown = 17;  //B0  manLiftDown
//const int downStopIn = 3;   //D0  liftAtBottom
const int downStopIn = 9;   //B5  liftAtBottom

//ERROR PIN
const int ERROR_PIN = 7;    //E6
 

//4 outputs
const int fwdOut = 13;      //C7  moveCartFwd
const int backOut = 5;      //C6  moveCartBack
//const int upOut = 15;       //B1  moveLiftUp
const int upOut = 6;       //D7  moveLiftUp
//const int downOut = 17;     //B0  moveLiftDown
const int downOut = 12;     //D6  moveLiftDown

/* Variables to hold the current and previous states */
uint16_t prevState = 0x0000;
uint16_t currState = 0x0000;


void setup() {
  Serial.begin(9600);
 
// Set ramp signals as inputs
  pinMode(fwdIn, INPUT);
  pinMode(backIn, INPUT);
  pinMode(fwdStopIn, INPUT);
  pinMode(backStopIn, INPUT);
  
// Set lift signals as inputs
  pinMode(manualUp, INPUT);
  pinMode(manualDown, INPUT);
  pinMode(downStopIn, INPUT);

// Set output pins:
  pinMode(ERROR_PIN,OUTPUT);
  pinMode(fwdOut,OUTPUT);
  pinMode(backOut,OUTPUT);
  pinMode(upOut, OUTPUT);
  pinMode(downOut, OUTPUT);

// Set the inputs as high (since active low)
  digitalWrite(fwdIn, HIGH);
  digitalWrite(backIn, HIGH);
  digitalWrite(fwdStopIn, HIGH);
  digitalWrite(backStopIn, HIGH);
  
  digitalWrite(manualUp, HIGH);
  digitalWrite(manualDown, HIGH);
  digitalWrite(downStopIn, HIGH);

// Set outputs as low initially
  digitalWrite(ERROR_PIN,LOW);
  digitalWrite(fwdOut, LOW);
  digitalWrite(backOut, LOW);
  digitalWrite(upOut, LOW);
  digitalWrite(downOut, LOW);
}

/* This function takes all of the inputs and adds it into 
   the state to be returned (currstate). All inputs are 
   active low. */
uint16_t checkInputs(){
  uint8_t tempState = 0x00;
  tempState |= (!digitalRead(fwdIn)) << FWD_IN_OFFSET;
  tempState |= (!digitalRead(backIn)) << BACK_IN_OFFSET;
  tempState |= (!digitalRead(fwdStopIn)) << FWD_STOP_IN;
  tempState |= (!digitalRead(backStopIn)) << BACK_STOP_IN;  
  tempState |= (!digitalRead(manualUp)) << UP_IN_OFFSET;
  tempState |= (!digitalRead(manualDown)) << DOWN_IN_OFFSET;
  tempState |= (!digitalRead(downStopIn)) << DOWN_STOP_IN;
  return tempState;
}

void loop() {
  prevState = currState;
  currState = checkInputs();
  Serial.print("State: ");
  Serial.println(currState);
  delay(100);

  switch (currState) {
  case 0x00:
    stopCarts();
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

  //case 0x05: //MOVE FWD, BUT FWD END IS ON
  case 0x45: // MOVE FWD, BUT FWD END IS ON
  case 0x4A: // MOVE BACKWARE, BUT BACK END IS ON
    stopCarts();
    break;

  case 0x42: //MOVE BACKWARD SWITCH IS ON
  case 0x46: //MOVE BACKWARD, FWD END IS ON
    movRev();
    break;

//adding new states
  case 0x14:  //MANUAL RAISE LIFT IS ON, CARTS IN FRONT
  case 0x18:  //MANUAL RAISE LIFT IS ON, CARTS IN BACK
  case 0x54:  //MANUAL RAISE LIFT IS ON, LIFT IS AT BOTTOM, CARTS IN FRONT
  case 0x58:  //MANUAL RAISE LIFT IS ON, LIFT IS AT BOTTOM, CARTS IN BACK
    movUp();
    break;
    
  case 0x04:  //CART IS RAISED, BUT NOT MOVING, CARTS IN FRONT
  case 0x08:  //CART IS RAISED, BUT NOT MOVING, CARTS IN BACK
    stopLift();
    break;
    
  case 0x24:  //MOVE DOWN SWITCH IS ON, LIFT IS NOT AT BOTTOM, CARTS IN FRONT
  case 0x28:  //MOVE DOWN SWITCH IS ON, LIFT IS NOT AT BOTTOM, CARTS IN BACK
    movDown();
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
  digitalWrite(backOut,LOW);
  digitalWrite(fwdOut,HIGH);
}

void stopCarts(){
  digitalWrite(fwdOut,LOW);
  digitalWrite(backOut,LOW);
}

void movRev(){
  digitalWrite(fwdOut,LOW);
  digitalWrite(backOut,HIGH); 
}

void movUp(){
  digitalWrite(downOut, LOW);
  digitalWrite(upOut, HIGH);
}

void stopLift(){
  digitalWrite(downOut, LOW);
  digitalWrite(upOut, LOW);
}

void movDown(){
  digitalWrite(upOut, LOW);
  digitalWrite(downOut, HIGH);
}

void error(){
  while (1){
    digitalWrite(fwdOut,LOW);
    digitalWrite(backOut,LOW);
    digitalWrite(upOut, LOW);
    digitalWrite(downOut, LOW);
    digitalWrite(ERROR_PIN,HIGH);
    Serial.println("ERROR");
    Serial.println(currState);
    delay(5000);
  }
}
