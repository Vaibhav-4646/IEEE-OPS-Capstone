#define PCB_1 16
#define PCB_2 14
#define PCB_FRONT 15
#define IN1 18
#define IN2 17
#define IN3 6
#define IN4 5
#define PWM1 22
#define PWM2 3

int zeroPCB1 = 0;
int zeroPCB2 = 0;
int zeroPCBFront = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(PCB_1, INPUT);
  pinMode(PCB_2, INPUT);
  pinMode(PCB_FRONT, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  int zeroConstant = 50000;
  
  for (int i = 0; i < zeroConstant; i++)
  {
      zeroPCB1 += analogRead(PCB_1);
      zeroPCB2 += analogRead(PCB_2);
      zeroPCBFront += analogRead(PCB_FRONT);
  }

  zeroPCB1 /= zeroConstant;
  zeroPCB2 /= zeroConstant;
  zeroPCBFront /= zeroConstant;

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(PWM1, HIGH);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  int averageLeft, averageRight, averageFront;

  for (int i = 0; i < 10; i ++)
  {
    averageLeft = analogRead(PCB_2);
    averageRight = analogRead(PCB_1);
    averageFront = analogRead(PCB_FRONT);
  }
  averageLeft/=10;
  averageRight/= 10;
  averageFront /= 10;
  
  int leftPCB = analogRead(PCB_2);
  int rightPCB = analogRead(PCB_1);
  int frontPCB = analogRead(PCB_FRONT);
  

  //Pseudocode:

  // if neither left nor right show something, default is to go straight
  // Maybe we do NOT need to zero it out! -> basically if it reads high, then 
  // there is a wall, so do NOT turn. And if it reads zero, then turn to that
  // if ALL three are high, then we are done.

  
  
 // Serial.println("LeftPCB = " + leftPCB - zeroPCB2);
  Serial.print("Front is ");
  Serial.println(frontPCB - zeroPCBFront);
  delay(100);
}
