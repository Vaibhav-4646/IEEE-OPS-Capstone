#define PCB_L 16
#define PCB_R 14
#define PCB_F 15
#define IN1 18        // LEFT MOTOR
#define IN2 17
#define IN3 6         // RIGHT MOTOR
#define IN4 5
#define PWM_L 22
#define PWM_R 3

double kp = .2;      // Proportional gain
double ki = 0;        // Integral gain 
double kd = 0.05;       // Derivative gain

int zeroPCB1 = 0;
int zeroPCB2 = 0;
int zeroPCBFront = 0;

int loopCounter = 0;

int right, left, front;

int leftSpeed;
int rightSpeed;

int rightDistance;
int leftDistance;

// Parameters for the controller
int rightError = 0;
int leftError = 0;
int prevRightError = 0;
int prevLeftError = 0;
int prevTotalError = 0;
int totalError = 0;
int tolerance = 0;
int sumError = 0;

double prevTime = 0;

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
  
  rightError = 0;
  leftError = 0;
  totalError = 0;
  prevRightError = 0;
  prevLeftError = 0;
  tolerance = 20;
  prevTime = millis();
}

void loop() {
  while(wallsPresent()) {
      moveStraight();
      Serial.print("Right: ");
      Serial.println(acquireSensor(PCB_R));
      Serial.print("Left: ");
      Serial.println(acquireSensor(PCB_L)); 
      Serial.print("Front: ");
      Serial.println(acquireSensor(PCB_F));
  }
  turn();
}

bool wallsPresent() {
  //Serial.println("In wallsPresent");
  right = acquireSensor(PCB_R);
  left = acquireSensor(PCB_L);
  front = acquireSensor(PCB_F);
  return front <= threshold_F*2.4 && front <= 730;
}

void moveStraight() {
  //Serial.println("In moveStraight");  
  loopCounter++;
  Serial.print("Loop counter is: ");
  Serial.println(loopCounter);
  rightDistance = right;
  leftDistance = left;
  int defSpeed = 175;
  rightSpeed = defSpeed;
  leftSpeed = defSpeed;
  rightError = threshold_R - rightDistance;
  Serial.print("Right Error is : ");
  Serial.println(rightError);
  leftError = threshold_L - leftDistance;
  Serial.print("Left Error is : ");
  Serial.println(leftError);

  if (right < threshold_R/2) {
    totalError = -leftError;
    Serial.println("No right wall");
  }
  else if (left < threshold_L/2) {
    totalError = rightError;
    Serial.println("No left wall");
  }
  else
    totalError = rightError - leftError;
  
  /* how long since the last calculated */
  double nowT = millis();       // current time
  double timeChange = (double)(nowT - prevTime);
  /* 
   *  computing working error values
   */
  sumError += (totalError * timeChange);
  double derivError = (totalError - prevTotalError);
  double PIDerrorSum = kp * totalError + ki * sumError + kd * derivError;
      
  if (totalError < -tolerance) {          // TODO: we need to change this part to now use I and D.
    leftSpeed -= abs(PIDerrorSum); 
    rightSpeed += abs(PIDerrorSum);
  } else if (totalError > tolerance) {      // if total error is negative, left error > right, which means its getting closer to the left wall
    leftSpeed += abs(PIDerrorSum);
    rightSpeed -= abs(PIDerrorSum);
  } else {
    rightSpeed = 220;
    leftSpeed = 220;
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

  Serial.print("Right motor speed is: ");
  Serial.println(rightSpeed);
  Serial.print("Left motor speed is: ");
  Serial.println(leftSpeed);
  digitalWrite(IN1, LOW);           //THIS MOVES FORWARD CORRECTLY. THE HIGH AND LOWS ARE CORRECT
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(PWM_R, rightSpeed);
  analogWrite(PWM_L, leftSpeed);

  prevLeftError = leftError;        // changing up the variables for the next iteration of the loop
  prevRightError = rightError;
  prevTotalError = totalError;
  prevTime = nowT;

//  if(loopCounter == 100)
//  {
//    Serial.print("The sum of the errors up till loop 100 is: ");        // if the error is larger and larger, then 
//                                                                        // that means its straying off the center more and more
//    Serial.println(sumError);
//    earlyStop();
//  }
  
}

void earlyStop()
{
  digitalWrite(IN1, HIGH); //Brake
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  Serial.println(" ");
  while (true) {}
}

void stopMoving() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

void turn() {
    //figures out which direction to turn, and then EXECUTES
    Serial.println("In turn");
    if (right < threshold_R/2) {
      turnRight();
    } else if (left < threshold_L/2) {
      turnLeft();
    }
}

void turnRight() {
  Serial.println("In turn Right");
  int prevLeft = left;
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(PWM_R, 255);
  analogWrite(PWM_L, 255);
  delay(60);
  Serial.print("Left sensor is: ");
  Serial.println(left);
  Serial.print("Beginning time: ");
  Serial.println(millis());
  int nowTime = millis();
  while ((left = acquireSensor(PCB_L)) < prevLeft-60 && nowTime-millis() < 2000) //Prevent an infinte loop using time
  {
    Serial.println(left);
  }
  Serial.print("Ending time: ");
  Serial.println(millis());
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(PWM_R, 0);
  analogWrite(PWM_L, 0);
  prevTime = millis();
}

void turnLeft() {
  Serial.println("In turn left");
  int prevRight = right;
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(PWM_R, 255);
  analogWrite(PWM_L, 255);
  delay(60);
  Serial.print("Right sensor is: ");
  Serial.println(right);
  Serial.print("Beginning time: ");
  Serial.println(millis());
  int nowTime = millis();
  while ((right = acquireSensor(PCB_R)) < prevRight-60 && nowTime-millis() < 2000) //Prevent an infinte loop using time
  {
    Serial.println(right);
  }
  Serial.print("Ending time: ");
  Serial.println(millis());
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(PWM_R, 0);
  analogWrite(PWM_L, 0);
  prevTime = millis();
}


int acquireSensor(int pin) {
  int sum = 0;
  int numSamples = 19;
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(pin);
  }
  return sum/numSamples;
}

