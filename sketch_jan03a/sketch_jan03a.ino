const int dirPinA = 12;
const int pwmPinA = 3;
const int brakePinA = 9;
const int dirPinB = 13;
const int pwmPinB = 11;
const int brakePinB = 8;

#define ENCA 4 // yellow encoder A
#define ENCB 5 // white encoder B

void setup() {
  pinMode(dirPinA, OUTPUT);
  pinMode(pwmPinA, OUTPUT);
  pinMode(brakePinA, OUTPUT);
  pinMode(dirPinB, OUTPUT);
  pinMode(pwmPinB, OUTPUT);
  pinMode(brakePinB, OUTPUT);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  
  Serial.begin(115200);
}

void loop() {
  int a = digitalRead(ENCA);
  int b = digitalRead(ENCB);
  
  if (Serial.available() > 0) {
    String commandLine = Serial.readStringUntil('\n');
    int separatorIndex = commandLine.indexOf(':');
    String command = commandLine.substring(0, separatorIndex);
    int speed = commandLine.substring(separatorIndex + 1).toInt();
    executeCommand(command, speed);
  }
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
