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

// Parameters for the controller
int rightError;
int leftError;
int prevRightError;
int prevLeftError;
int prevTotalError;
int totalError;
int tolerance;
int sumError;

int prevTime;
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

int threshold_R, threshold_L, threshold_F;

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
  threshold_F = zeroPCBFront;
  
  Serial.println(threshold_L);
  Serial.println(threshold_R);
  Serial.println(threshold_F);
  
  time = 100;
  rightError = 0;
  leftError = 0;
  totalError = 0;
  prevRightError = 0;
  prevLeftError = 0;
  tolerance = 20;
  //TODO: initialize proportional and integral error  (We don't need to do this)
}

void loop() {
  Serial.print("Right: ");
  Serial.println(acquireSensor(PCB_R));
  Serial.print("Left: ");
  Serial.println(acquireSensor(PCB_L)); 
  Serial.print("Front: ");
  Serial.println(acquireSensor(PCB_F));
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
  if (left < threshold_L-50)
    return false;
  if (right < threshold_R-50)
    return false;
  return true;
}

void moveStraight() {
  rightDistance = acquireSensor(PCB_R);
  leftDistance = acquireSensor(PCB_L);

  rightError = threshold_R - rightDistance;
  leftError = threshold_L - leftDistance;
  totalError = rightError - leftError;
  
  /* how long since the last calculated */
  unsigned long now = millis();       // current time
  double timeChange = (double)(now - prevTime);
  
  /* 
   *  computing working error values
   */
  sumError += (totalError * timeChange);
  double derivError = (totalError - prevTotalError)/timeChange;

  double PIDerror = totalError + sumError + derivError;
  double PIDerrorSum = kp * totalError + ki * sumError + kd * derivError;
      
  if (PIDerror < -1 * tolerance) {          // TODO: we need to change this part to now use I and D.
    rightSpeed -= abs(PIDerrorSum); 
    leftSpeed += abs(PIDerrorSum);
  } else if (PIDerror > tolerance) {      // if total error is negative, left error > right, which means its getting closer to the left wall
    rightSpeed += abs(PIDerrorSum);
    leftSpeed -= abs(PIDerrorSum);
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

  digitalWrite(IN1, LOW);           //THIS MOVES FORWARD CORRECTLY. THE HIGH AND LOWS ARE CORRECT
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(PWM_R, rightSpeed);
  analogWrite(PWM_L, leftSpeed);

  prevLeftError = leftError;        // changing up the variables for the next iteration of the loop
  prevRightError = rightError;
  prevTotalError = totalError;
  prevTime = now;
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
    if (right < threshold_R-50) {
      turnRight();
    } else if (left < threshold_L-50) {
      turnLeft();
    }
}

void turnLeft() {
  const int turnTime = 250;
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(PWM_R, 200);      // we have to figure out what PWM to do these at
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

