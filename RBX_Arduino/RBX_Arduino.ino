/*
 Bit 0 = fwdIn
 Bit 1 = backIn
 Bit 2 = fwdStopIn
 Bit 3 = backStopIn  
 Bit 4 = manual up
 Bit 5 = manual down
 Bit 6 = down endstop
 Bit 7 = something
 Bit 8 = unassigned
 Bit 9 = unassigned
 Bit 10 = unassigned
 Bit 11 = unassigned
 Bit 12 = unassigned
 Bit 13 = unassigned
 Bit 14 = unassigned
 Bit 15 = unassigned
 */
 
/*
OUTPUTS:
   1. moveFwd
   2. movRev
   3. liftUp
   4. liftDown
 */
const int FWD_IN_OFFSET = 0;
const int BACK_IN_OFFSET = 1;
const int FWD_STOP_IN = 2;
const int BACK_STOP_IN = 3;
const int UP_IN_OFFSET = 4;
const int DOWN_IN_OFFSET = 5;
const int DOWN_STOP_IN = 6;

//4 inputs for carts
const int fwdIn = 16;      //B2 
const int backIn = 14;     //B3 
const int fwdStopIn = 4;   //D4
const int backStopIn = 10; //B6

//3 inputs for lift
const int manualUp = 15;    //B1  
const int manualDown = 17;  //B0
const int downStopIn = 9;   //B5

//ERROR PIN
const int ERROR_PIN = 7;    //E6

//2 outputs for carts
const int fwdOut = 13;      //C7
const int backOut = 5;      //C6

//2 outputs for lift
const int upOut = 6;     //D7
const int downOut = 12;  //D6

uint16_t prevState = 0x00;
uint16_t currState = 0x00;


void setup() {
  Serial.begin(9600);

  pinMode(fwdIn, INPUT);
  pinMode(backIn, INPUT);
  pinMode(fwdStopIn, INPUT);
  pinMode(backStopIn, INPUT);
  pinMode(manualUp, INPUT);
  pinMode(manualDown, INPUT);
  pinMode(downStopIn, INPUT);

  pinMode(ERROR_PIN,OUTPUT);
  pinMode(fwdOut,OUTPUT);
  pinMode(backOut,OUTPUT);
  pinMode(upOut, OUTPUT);
  pinMode(downOut, OUTPUT);

  digitalWrite(fwdIn, HIGH);
  digitalWrite(backIn, HIGH);
  digitalWrite(fwdStopIn, HIGH);
  digitalWrite(backStopIn, HIGH);
  digitalWrite(manualUp, HIGH);
  digitalWrite(manualDown, HIGH);
  digitalWrite(downStopIn, HIGH);

  digitalWrite(ERROR_PIN,LOW);
  digitalWrite(fwdOut, LOW);
  digitalWrite(backOut, LOW);
  digitalWrite(upOut, LOW);
  digitalWrite(downOut, LOW);
}

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
    break;

  case 0x41: //MOVE FWD SWITCH IS ON
  case 0x49: //MOVE FWD, BACK END IS ON
    movFwd();
    break;

  case 0x45: //MOVE FWD, BUT FWD END IS ON
    stopCarts();
    break;

  case 0x42: //MOVE BACKWARD SWITCH IS ON
  case 0x46: //MOVE BACKWARD, FWD END IS ON
    movRev();
    break;

  case 0x4A:  //MOVE BACKWARD, BUT BACK END IS ON
    stopCarts();
    break;
    
  case 0x14:  //MOVE UP SWITCH IS ON
  case 0x18:
  case 0x54:
  case 0x58:
    movUp();
    break;
    
  case 0x04:  //CART IS RAISED, BUT NOT MOVING
  case 0x08:
    stopLift();
    break;
    
  case 0x24:  //MOVE DOWN SWITCH IS ON
  case 0x28:
    movDown();
    break;
    
  case 0x64:  //MOVE DOWN, BUT DOWN ENDSTOP IS ON
  case 0x68:
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
    delay(5000);
  }
}
