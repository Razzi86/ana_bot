const int dirPinA = 12;
const int pwmPinA = 3;
const int brakePinA = 9;
const int dirPinB = 13;
const int pwmPinB = 11;
const int brakePinB = 8;

int pos_front = 0;
int pos_back = 0;

#define FRONT_A 0 // yellow
#define FRONT_B 1 // white 
#define BACK_A 2 // yellow
#define BACK_B 7 // white

void setup() {
  pinMode(dirPinA, OUTPUT);
  pinMode(pwmPinA, OUTPUT);
  pinMode(brakePinA, OUTPUT);
  pinMode(dirPinB, OUTPUT);
  pinMode(pwmPinB, OUTPUT);
  pinMode(brakePinB, OUTPUT);

  pinMode(FRONT_A,INPUT);
  pinMode(FRONT_B,INPUT);
  pinMode(BACK_A,INPUT);
  pinMode(BACK_B,INPUT);
  
  attachInterrupt(digitalPinToInterrupt(FRONT_A),readEncoder_front,RISING); // if A rising
  attachInterrupt(digitalPinToInterrupt(BACK_A),readEncoder_back,RISING); // if A rising
  
  Serial.begin(115200);
}

void loop() {
  
  int front_a = digitalRead(FRONT_A);
  int front_b = digitalRead(FRONT_B);
  int back_a = digitalRead(BACK_A);
  int back_b = digitalRead(BACK_B);

  // stopMotor();
  
  // READING FROM ENCODER MOTOR
  // Serial.print("front a,b: ");
  // Serial.print(front_a*5);
  // Serial.print(" ");
  // Serial.print(front_b*5);
  // Serial.println("back a,b: ");
  // Serial.print(front_a*5);
  // Serial.print(" ");

  // MEASURE POSTION FROM ENCODER MOTOR
//  Serial.print(pos_front);
//  Serial.print(" ");
//  Serial.println(pos_back);

  static unsigned long lastSendTime = 0;
  unsigned long sendInterval = 25; // milliseconds

  if (millis() - lastSendTime > sendInterval) {
    Serial.print("Front pos: ");
    Serial.print(pos_front);
    Serial.print( "Back pos: ");
    Serial.println(pos_back);

    lastSendTime = millis();
  }
  
  if (Serial.available() > 0) {
    String commandLine = Serial.readStringUntil('\n');
    int separatorIndex = commandLine.indexOf(':');
    String command = commandLine.substring(0, separatorIndex);
    int speed = commandLine.substring(separatorIndex + 1).toInt();
    executeCommand(command, speed);
  }
}

void readEncoder_front(){
  int b_f = digitalRead(FRONT_B);
  if(b_f>0){
    pos_front++; // if b rises before a
  }
  else{
    pos_front--; // if b rises after a
  }
}

void readEncoder_back(){
  int b_b = digitalRead(BACK_B);
  if(b_b>0){
    pos_back++; // if b rises before a
  }
  else{
    pos_back--; // if b rises after a
  }
}

void stopMotors() {
  digitalWrite(pwmPinA, LOW);
  digitalWrite(brakePinA, HIGH);
  digitalWrite(pwmPinB, LOW);
  digitalWrite(brakePinB, HIGH);
}


void executeCommand(String command, int speed) {
  if (command == "start_forward") {
    moveMotor(HIGH, speed);
  } else if (command == "start_backward") {
    moveMotor(LOW, speed);
  } else if (command == "stop") {
    stopMotor();
  } else if (command == "start_forward_slow") {
    moveMotor(HIGH, speed / 2); // Adjust this as needed
  }
}

void moveMotor(bool direction, int speed) {
  digitalWrite(brakePinA, LOW);
  digitalWrite(brakePinB, LOW);
  digitalWrite(dirPinA, direction);
  digitalWrite(dirPinB, direction);
  analogWrite(pwmPinA, speed);
  analogWrite(pwmPinB, speed);
}

void stopMotor() {
  digitalWrite(brakePinA, HIGH);
  digitalWrite(brakePinB, HIGH);
  analogWrite(pwmPinA, 0);
  analogWrite(pwmPinB, 0);
}
