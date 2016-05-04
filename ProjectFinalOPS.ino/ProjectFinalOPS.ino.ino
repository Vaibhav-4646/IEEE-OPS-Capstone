#define PCB_L 16
#define PCB_R 14
#define PCB_F 15
#define IN1 18
#define IN2 17
#define IN3 6
#define IN4 5
#define PWM_L 22
#define PWM_R 3

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
double kp = 5; // Proportional gain
double ki = 1; // Integral gain

// Functions to move and decide how to move
int acquireSensor(int pin);
void moveStraight();
void turnRight();
void turnLeft();
void stopMoving();
bool wallsPresent();
void turn();

int threshold_R, threshold_L;

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

  threshold_L = zeroPCB1;
  threshold_R = zeroPCB2;
  Serial.println(threshold_L);
  Serial.println(threshold_R);
  time = 100;
  rightError = 0;
  leftError = 0;
  totalError = 0;
  prevRightError = 0;
  prevLeftError = 0;
  //TODO: initialize proportional and integral error  
}

void loop() {
  Serial.print("Right: ");
  Serial.println(acquireSensor(PCB_R));
  Serial.print("Left: ");
  Serial.println(acquireSensor(PCB_L)); 
  while(wallsPresent()) {
    moveStraight();
  }
  stopMoving();
  turn();
  delay(time);
}

bool wallsPresent() {
  int right = acquireSensor(PCB_R);
  int left = acquireSensor(PCB_L);
  int front = acquireSensor(PCB_F);
  if (left < threshold_L-100)
    return false;
  if (right < threshold_R-100)
    return false;
  return true;
}

void moveStraight() {
  rightDistance = acquireSensor(PCB_R);
  leftDistance = acquireSensor(PCB_L);

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

  digitalWrite(IN1, LOW); //THIS MOVES FORWARD CORRECTLY. THE HIGH AND LOWS ARE CORRECT
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(PWM_R, rightSpeed);
  analogWrite(PWM_L, leftSpeed);
  
}

void stopMoving() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

void turn() {
    //figures out which direction to turn, and then EXECUTES
    int right = acquireSensor(PCB_R);
    int left = acquireSensor(PCB_L);
    if (right < threshold_R-100) {
      turnRight();
    } else if (left < threshold_L-100) {
      turnLeft();
    }
}

void turnLeft() {
  const int turnTime = 250;
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(PWM_R, 200);
  analogWrite(PWM_L, 200);
  delay(turnTime);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(PWM_R, 0);
  analogWrite(PWM_L, 0);
}

void turnRight() {
  const int turnTime = 250;
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(PWM_R, 200);
  analogWrite(PWM_L, 200);
  delay(turnTime);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(PWM_R, 0);
  analogWrite(PWM_L, 0);
}

int acquireSensor(int pin) {
  int sum = 0;
  int numSamples = 600;
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(pin);
  }
  return sum/numSamples;
}

