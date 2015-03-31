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
const int manCartFwd = 16;   //B2
const int manCartBack = 14;  //B3 
const int cartAtFront = 4;   //D4
const int cartAtBack = 10;   //B6  

//3 inputs for lift
const int manLiftUp = 15;    //B1
const int manLiftDown = 17;  //B0
const int liftAtBottom = 9;  //B5  

//ERROR PIN
const int ERROR_PIN = 7;    //E6
 

//4 outputs
const int moveCartFwd = 13;   //C7
const int moveCartBack = 5;   //C6
const int moveLiftUp = 6;     //D7
const int moveLiftDown = 12;  //D6

/* Variables to hold the current and previous states */
uint16_t prevState = 0x0000;
uint16_t currState = 0x0000;


void setup() {
  Serial.begin(9600);
 
// Set ramp signals as inputs
  pinMode(manCartFwd, INPUT);
  pinMode(manCartBack, INPUT);
  pinMode(cartAtFront, INPUT);
  pinMode(cartAtBack, INPUT);
  
// Set lift signals as inputs
  pinMode(manLiftUp, INPUT);
  pinMode(manLiftDown, INPUT);
  pinMode(liftAtBottom, INPUT);

// Set output pins:
  pinMode(ERROR_PIN,OUTPUT);
  pinMode(moveCartFwd,OUTPUT);
  pinMode(moveCartBack,OUTPUT);
  pinMode(moveLiftUp, OUTPUT);
  pinMode(moveLiftDown, OUTPUT);

// Set the inputs as high (since active low)
  digitalWrite(manCartFwd, HIGH);
  digitalWrite(manCartBack, HIGH);
  digitalWrite(cartAtFront, HIGH);
  digitalWrite(cartAtBack, HIGH);
  
  digitalWrite(manLiftUp, HIGH);
  digitalWrite(manLiftDown, HIGH);
  digitalWrite(liftAtBottom, HIGH);

// Set outputs as low initially
  digitalWrite(ERROR_PIN,LOW);
  digitalWrite(moveCartFwd, LOW);
  digitalWrite(moveCartBack, LOW);
  digitalWrite(moveLiftUp, LOW);
  digitalWrite(moveLiftDown, LOW);
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
  digitalWrite(moveCartBack,LOW);
  digitalWrite(moveCartFwd,HIGH);
}

void stopCarts(){
  digitalWrite(moveCartFwd,LOW);
  digitalWrite(moveCartBack,LOW);
}

void movRev(){
  digitalWrite(moveCartFwd,LOW);
  digitalWrite(moveCartBack,HIGH); 
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

void error(){
  while (1){
    digitalWrite(moveCartFwd,LOW);
    digitalWrite(moveCartBack,LOW);
    digitalWrite(moveLiftUp, LOW);
    digitalWrite(moveLiftDown, LOW);
    digitalWrite(ERROR_PIN,HIGH);
    Serial.println("ERROR");
    Serial.println(currState);
    delay(5000);
  }
}
