/*
 Bit 0 = fwdIn        // manual carts forward --> manCartFwd
 Bit 1 = backIn       // manual carts back  -->  manCartBack
 Bit 2 = fwdStopIn    // button at front of ramp  --> cartAtFront
 Bit 3 = backStopIn   // button at back of ramp  --> cartAtBack
 Bit 4 = manual up    // manual lift up  --> manLiftUp
 Bit 5 = manual down  // manual lift down  --> manLiftDown
 Bit 6 = down endstop // lift down button  --> liftAtBottom
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
   1. moveFwdOut  //moving carts forward  --> moveCartFwd
   2. movRevOut  // moving carts backward  --> moveCartBack
   3. liftUpOut  // moving lift up  --> moveLiftUp
   4. liftDownOut  // moving lift down  --> moveLiftDown
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
const int manualUp = 6;     //D7  manLiftUp
const int manualDown = 12;  //D6  manLiftDown
const int downStopIn = 3;   //D0  liftAtBottom

//ERROR PIN
const int ERROR_PIN = 7;    //E6

//4 outputs
const int fwdOut = 13;      //C7  moveCartFwd
const int backOut = 5;      //C6  moveCartBack
const int upOut = 15;       //B1  moveLiftUp
const int downOut = 17;     //B0  moveLiftDown

uint16_t prevState = 0x00;
uint16_t currState = 0x00;


void setup() {
  Serial.begin(9600);

  pinMode(fwdIn, INPUT);
  pinMode(backIn, INPUT);
  pinMode(fwdStopIn, INPUT);
  pinMode(backStopIn, INPUT);

  pinMode(ERROR_PIN,OUTPUT);
  pinMode(fwdOut,OUTPUT);
  pinMode(backOut,OUTPUT);
  pinMode(upOut, OUTPUT);
  pinMode(downOut, OUTPUT);

  digitalWrite(fwdIn, HIGH);
  digitalWrite(backIn, HIGH);
  digitalWrite(fwdStopIn, HIGH);
  digitalWrite(backStopIn, HIGH);

  digitalWrite(ERROR_PIN,LOW);
  digitalWrite(fwdOut, LOW);
  digitalWrite(backOut, LOW);
}

uint16_t checkInputs(){
  uint8_t tempState = 0x00;
  tempState |= (!digitalRead(fwdIn)) << FWD_IN_OFFSET;
  tempState |= (!digitalRead(backIn)) << BACK_IN_OFFSET;
  tempState |= (!digitalRead(fwdStopIn)) << FWD_STOP_IN;
  tempState |= (!digitalRead(backStopIn)) << BACK_STOP_IN;  
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

  case 0x05: //MOVE FWD, BUT FWD END IS ON
    stopCarts();
    break;

  case 0x42: //MOVE BACKWARD SWITCH IS ON
  case 0x46: //MOVE BACKWARD, FWD END IS ON
    movRev();
    break;

  case 0x0A:  //MOVE BACKWARD, BUT BACK END IS ON
    stopCarts();
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
    delay(5000);
  }
}
