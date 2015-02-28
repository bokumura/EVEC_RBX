/*
 Bit 0 = fwdIn
 Bit 1 = backIn
 Bit 2 = fwdStopIn
 Bit 3 = backStopIn  
 Bit 4 = manual up
 Bit 5 = manual down
 Bit 6 = down endstop
 Bit 7 = unassigned
 Bit 8 = unassigned
 Bit 9 = unassigned
 Bit 10 = unassigned
 Bit 11 = unassigned
 Bit 12 = unassigned
 Bit 13 = unassigned
 Bit 14 = unassigned
 Bit 15 = unassigned
 */
<<<<<<< Updated upstream

=======
 
 /*
  OUTPUTS:
   1. moveFwd
   2. movRev
   3. liftUp
   4. liftDown
 */
 
>>>>>>> Stashed changes
const int FWD_IN_OFFSET = 0;
const int BACK_IN_OFFSET = 1;
const int FWD_STOP_IN = 2;
const int BACK_STOP_IN = 3;
const int UP_IN_OFFSET = 4;
const int DOWN_IN_OFFSET = 5;
const int DOWN_STOP_IN = 6;

//4 inputs for ramp
const int fwdIn = 16;      //B2 
const int backIn = 14;     //B3 
const int fwdStopIn = 4;   //D4
const int backStopIn = 10; //B6

<<<<<<< Updated upstream
//3 inputs for lift
=======
>>>>>>> Stashed changes
const int manualUp = 6;     //D7  
const int manualDown = 12;  //D6
const int downStopIn = 3;   //D0

//ERROR PIN
const int ERROR_PIN = 7;    //E6

//2 outputs
const int fwdOut = 13;      //C7
const int backOut = 5;      //C6

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
    break;

  case 0x01: //MOVE FWD SWITCH IS ON
    movFwd();
    break;

  case 0x05: //MOVE FWD, BUT FWD END IS ON
    stopCarts();
    break;

  case 0x02: //MOVE BACKWARD SWITCH IS ON
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

void error(){
  while (1){
    digitalWrite(fwdOut,LOW);
    digitalWrite(backOut,LOW);
    digitalWrite(ERROR_PIN,HIGH);
    Serial.println("ERROR");
    delay(5000);
  }
}
