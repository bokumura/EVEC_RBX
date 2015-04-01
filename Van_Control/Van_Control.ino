/*
Bit 0 = manActuatorsEngage
Bit 1 = manActuatorsDisengage
Bit 2 = actuatorsEngaged
Bit 3 = actuatorsDisengaged
*/

/*
OUTPUTS:
  1. movEngageActuators
  2. movDisengageActuators
*/

//Offsets of signals in curState
const int MAN_ACTUATORS_ENGAGE_OFFSET = 0;
const int MAN_ACTUATORS_DISENGAGE_OFFSET = 1;
const int ACTUATORS_ENGAGED_OFFSET = 2;
const int ACTUATORS_DISENGAGED_OFFSET = 3;

//Threshholds for actuator placement
const int ACTUATORS_DISENGAGED_THRESHHOLD = 177;
const int ACTUATORS_ENGAGED_THRESHHOLD = 8;

//Inputs for van
const int manActuatorsEngage = 16;  //B2 = Digital pin 16
const int manActuatorsDisengage = 15  //B1 = Digital pin 15
const int frontActuatorLocationPin = 0;  //F7 = Analog pin 0
const int rearActuatorLocationPin = 1;  //F6 = Analog pin 1

//ERROR PIN
const int ERROR_PIN = 7  //E6

//Outputs
const int movActuatorsEngage = 6  //D7 = Digital pin 6
const int movActuatorsDisengage = 12  //D6 = Digital pin 12

//Current State
uint8_t currState = 0x00;

void setup() {
  Serial.begin(9600);
  
  //Set actuator signals as inputs
  pinMode(manActuatorsEngage, INPUT);
  pinMode(manActuatorsDisengage, INPUT);
  pinMode(frontActuatorLocationPin, INPUT);
  pinMode(rearActuatorLocationPin, INPUT);
  
  //Set output pins
  pinMode(ERROR_PIN, OUTPUT);
  pinMode(movActuatorsEngage, OUTPUT);
  pinMode(movActuatorsDisengage, OUTPUT);
  
  //Set digital inputs as high (since active low)
  digitalWrite(manActuatorsEngage, HIGH);
  digitalWrite(manActuatorsDisengage, HIGH);
  
  //Initialize outputs to low
  digitalWrite(ERROR_PIN, LOW);
  digitalWrite(movEngageActuators, LOW);
  digitalWrite(movDisengageActuators, LOW);
}

/* This function takes all inputs and ORs them onto the proper
    position on the state to be returned (curstate). Digital inputs
    are active low, so they are inverted. Analog inputs are compared
    to their threshholds and set accordingly. */
uint8_t checkInputs() {
  uint8_t tempState = 0x00;
  tempState |= (!digitalRead(manActuatorsEngage)) << MAN_ACTUATORS_ENGAGE_OFFSET;
  tempState |= (!digitalRead(manActuatorsDisengage)) << MAN_ACTUATORS_DISENGAGE_OFFSET;
  tempState |= ((analogRead(frontActuatorLocationPin) > ACTUATORS_ENGAGED_THRESHHOLD) 
    && (analogRead(rearActuatorLocationPin) > ACTUATORS_ENGAGED_THRESHHOLD))
    << ACTUATORS_ENGAGED_OFFSET;
  tempState |= ((analogRead(frontActuatorLocationPin) < ACTUATORS_DISENGAGED_THRESHHOLD)
    && (analogRead(rearActuatorLocationPin) < ACTUATORS_DISENGAGED_THRESHHOLD))
    << ACTUATORS_DISENGAGED_OFFSET;
  return tempState;
}

void loop() {
  currState = checkInputs();
  Serial.print("State: ");
  Serial.println(currState);
  delay(100);
  
  switch (currState) {
  case 0x00:  //EVERYTHING IS OFF
  case 0x04:  //ACTUATORS ENGAGED, NOT MOVING
  case 0x05:  //TRYING TO ENGAGE ACTUATORS, BUT THEY ARE ALREADY ENGAGED
  case 0x08:  //ACTUATORS DISENGAGED, NOT MOVING
  case 0x0A:  //TRYING TO DISENGAGE ACTUATORS, BUT THEY ARE ALREADY DISENGAGED
    stopActuators();
    break;
    
  case 0x01:  //ENGAGING ACTUATORS, CURRENTLY NEITHER ENGAGED NOR DISENGAGED
  case 0x09:  //ENGAGING ACTUATORS, CURRENTLY DISENGAGED
    engageActuators();
    break;
    
  case 0x02:  //DISENGAGING ACTUATORS, CURRENTLY NEIGHER ENGAGED NOR DISENGAGED
  case 0x06:  //DISENGAGING ACTUATORS, CURRENTLY ENGAGED
    disengageActuators();
    break;
    
  default:  //LOL GG DONE MESSED UP SON (as Brandon would say)
    error();
    break;
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
  digitalWrite(movActuatorsEngage, LOW);
  digitalWrite(movActuatorsDisengage, LOW);
}
