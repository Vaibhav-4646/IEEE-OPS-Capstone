#define PCB_1 23
#define PCB_2 20

int zeroPCB1;
int zeroPCB2;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(PCB_1, INPUT);
  pinMode(PCB_2, INPUT);

  int zeroConstant = 50000;
  
  for (int i = 0; i < zeroConstant; i++)
  {
      zeroPCB1 += analogRead(PCB_1);
      zeroPCB2 += analogRead(PCB_2);
  }

  zeroPCB1 /= zeroConstant;
  zeroPCB2 /= zeroConstant;
}

void loop() {
  // put your main code here, to run repeatedly:

  int averageLeft, averageRight;

  for (int i = 0; i < 10; i ++)
  {
    averageLeft = analogRead(PCB_2);
    averageRight = analogRead(PCB_1);
  }
  averageLeft/=10;
  averageRight/= 10;
  
  int leftPCB = analogRead(PCB_2);
  int rightPCB = analogRead(PCB_1);

  //Pseudocode:

  // if neither left nor right show something, default is to go straight
  // Maybe we do NOT need to zero it out! -> basically if it reads high, then 
  // there is a wall, so do NOT turn. And if it reads zero, then turn to that
  // if ALL three are high, then we are done.

  
  
 // Serial.println("LeftPCB = " + leftPCB - zeroPCB2);
  Serial.print("Right is");
  Serial.println(rightPCB - zeroPCB1);
  delay(100);
}
