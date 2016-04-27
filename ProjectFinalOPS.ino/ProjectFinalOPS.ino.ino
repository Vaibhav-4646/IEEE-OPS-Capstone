#define PCB_L 16
#define PCB_R 14
#define PCB_F 15
#define IN1 18
#define IN2 17
#define IN3 6
#define IN4 5
#define PWM_L 3
#define PWM_R 22

int zeroPCB1 = 0;
int zeroPCB2 = 0;
int zeroPCBFront = 0;

int leftSpeed;
int rightSpeed;

int rightDistance;
int leftDistance;

int rightBaseline;
int leftBaseline;

// Parameters for the controller
int rightError;
int leftError;
int prevRightError;
int prevLeftError;
int totalError;
int tolerance;

int time;  // Time between control loop iterations
int integral;
double kp; // Proportional gain
double ki; // Integral gain

// Functions to move and decide how to move
int acquireSensor(int pin);
void moveStraight();
void turnRight();
void turnLeft();
void stopMoving();
bool wallsPresent();
void turn();

int threshold;

void setup() {
  Serial.begin(9600);
  pinMode(PCB_L, INPUT);
  pinMode(PCB_R, INPUT);
  pinMode(PCB_F, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  int zeroConstant = 50000;
  
  for (int i = 0; i < zeroConstant; i++) {
      zeroPCB1 += analogRead(PCB_L);
      zeroPCB2 += analogRead(PCB_R);
      zeroPCBFront += analogRead(PCB_F);
  }

  zeroPCB1 /= zeroConstant;
  zeroPCB2 /= zeroConstant;
  zeroPCBFront /= zeroConstant;

  threshold = zeroPCB1;
  Serial.println(threshold);
  kp = 1;
  ki = 1;
  time = 100;

  //TODO: initialize proportional and integral error  
}

void loop() {
  while(wallsPresent()) {
    moveStraight();
  }
  stopMoving();
  turn();
  Serial.println(analogRead(PCB_L));
  delay(time);
}

bool wallsPresent() {
  //Checks if rodent can move forward
  int right = acquireSensor(PCB_R);
  int left = acquireSensor(PCB_L);
  int front = acquireSensor(PCB_F);
//  int threshold = 500; //TODO: Find the threshold
return true;
//  return (right > threshold-100 && left > threshold-100);
}

void moveStraight() {
  rightDistance = acquireSensor(PCB_L); //TODO: Should this be analogRead or acquireSensor?
  leftDistance = acquireSensor(PCB_R);

  rightError = rightBaseline - rightDistance;
  leftError = leftBaseline - leftDistance;
  totalError = rightError - leftError;

  if (totalError < -1 * tolerance) {
    rightSpeed -= abs(kp * totalError);
    leftSpeed += abs(kp * totalError);
  } else if (totalError > tolerance) {
    rightSpeed += abs(kp * totalError);
    leftSpeed -= abs(kp * totalError);
  } else {
    rightSpeed = 255;
    leftSpeed = 255;
  }

  if (rightSpeed < 0) {
    rightSpeed = 0;
  } else if (rightSpeed > 255) {
    rightSpeed = 255;
  }

  if (leftSpeed < 0) {
    leftSpeed = 0;
  } else if (leftSpeed > 255) {
    leftSpeed = 255;
  }

  digitalWrite(IN1, HIGH); //TODO: MIGHT NEED TO SWITCH THE HIGH/LOW
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, HIGH); //TODO: MIGHT NEED TO SWITCH THE HIGH/LOW
  digitalWrite(IN4, LOW);
  analogWrite(PWM_R, rightSpeed);
  analogWrite(PWM_R, leftSpeed);
  
}

void stopMoving() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

void turn() {
    //figures out which direction to turn, and then EXECUTES
    
}

void turnRight() {
  const int turnTime = 250;
  digitalWrite(IN1, HIGH); //TODO: MIGHT NEED TO SWITCH THE HIGH/LOW
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW); //TODO: MIGHT NEED TO SWITCH THE HIGH/LOW
  digitalWrite(IN4, HIGH);
  delay(turnTime);
  moveStraight();
}

void turnLeft() {
  const int turnTime = 250;
  digitalWrite(IN1, LOW); //TODO: MIGHT NEED TO SWITCH THE HIGH/LOW
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, HIGH); //TODO: MIGHT NEED TO SWITCH THE HIGH/LOW
  digitalWrite(IN4, LOW);
  delay(turnTime);
  moveStraight();
}

int acquireSensor(int pin) {
  int sum = 0;
  int numSamples = 20; //TODO: What is this actually supposed to be?
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(pin);
  }
  return sum/numSamples;
}

