//Variables for motor
int counterRIGHT = 0; //counter variable to track position of knob
int counterLEFT = 0; //counter variable to track position of knob

int currentStateARIGHT; //tracks the current state of A/CLK
int currentStateALEFT; //variable to store previous state of A/CLK

int previousStateARIGHT; //tracks the current state of A/CLK
int previousStateALEFT; //variable to store previous state of A/CLK

int currentStateBRIGHT; //tracks current state of B/DT
int currentStateBLEFT; //tracks current state of B/DT

double radianCountRIGHT;
double radianCountLEFT;

double pi = 3.14159;

int currentCountRIGHT;
int currentCountLEFT;

unsigned long timeNow;
double currentRadianRIGHT;
double currentRadianLEFT;

int i = 0;

double r = 0.075; // Radius of the wheel
double d = .36; // Distance Between Wheels

double preRho;
double prePhi;

double Va1;
double Va2;
double Va;
double deltaVa;

int debounceTime = 0; //debounce time. set to 0 ms.
int countsPerRevolution = 1600; //1600 counts per revolution.
int motorPWM;
double targetAngle;
double targetDistance;
double targetPhi, targetRho;
double errorPhi;
double errorRho;
double thetaDotRIGHT, thetaDotLEFT;

unsigned long pastMillisRIGHT = 0;
unsigned long pastMillisLEFT = 0;

double preRadianCountRIGHT = 0;
double preRadianCountLEFT = 0;

double currentRadianCountRIGHT, currentRadianCountLEFT;
double currentRhoDot, currentPhiDot;
double currentPhi = 0;
double currentDistance = 0;
float distanceInterval;
float angleInterval;

unsigned long previousMillisRIGHT = 0; //variable to track previous reading of the millis() function.
unsigned long previousMillisLEFT = 0;
unsigned long prevMillis = 0;

unsigned long previousMillisAngle;
unsigned long previousMillisDistance;
unsigned long currentMillisDistance;
unsigned long currentMillisAngle;

void encoderISRLEFT(); //function prototype for the ISR.void encoderISR(); //function prototype for the ISR.
void encoderISRRIGHT();
void turn(double);
void rho(double);
double TsRho = 0;
double TcRho = millis();

double deltaV;

double Kp = 4.2472; // porportion
double Ki = .024677; // integral number

double u1;
double u2;
double Ts;
double Tc;
double I1;
double I2;

double secIntervalLEFT;
double secIntervalRIGHT;


void setup() {
  Tc = 0;
  I1 = 0;
  I2 = 0;
  pinMode(2, INPUT); //motor 1 CLK or A
  pinMode(5, INPUT); //motor 1 DT or B
  pinMode(3, INPUT); //motor 2 CLK or A
  pinMode(6, INPUT); //motor 2 DT or B
  previousStateARIGHT = digitalRead(2); //motor 1 reads in the current state of A/CLK to start off the program.
  previousStateALEFT = digitalRead(3); //motor 2 reads in the current state of A/CLK to start off the program.
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  pinMode(7, OUTPUT); //Motor 1 Voltage Sign RIGHT used to be 8
  pinMode(9, OUTPUT); //Motor 1 Voltage RIGHT
  pinMode(8, OUTPUT); //Motor 2 Voltage Sign  LEFT used to be 7
  pinMode(10, OUTPUT); //Motor 2 Voltage LEFT
  digitalWrite(7, LOW); //LOW is forward for Motor 1
  digitalWrite(8, HIGH); //HIGH is forward for Motor 2
  pinMode(13, OUTPUT);
  Serial.begin(250000);


  attachInterrupt(1, encoderISRLEFT, CHANGE); 
  attachInterrupt(0, encoderISRRIGHT, CHANGE); //attach an interrupt to pin 2 (interrupt 0) to call the function encoderISR when any change happens in pin 2.
}

void loop() {
  
  timeNow = millis();
  targetAngle = 0;
  targetDistance = 1;
  rho(targetDistance);
  //turn(targetAngle);
  errorPhi = 0;
  
  Va1 = (double)((deltaV + errorPhi)/2);
  Va2 = (double)((deltaV - errorPhi)/2);
  //Serial.print(deltaV);
  //Serial.print(" ");
  //Serial.print(currentPhi);
  //Serial.print(" ");
  Serial.print(Va1);
  Serial.print(" ");
  Serial.print(Va2);
  Serial.print(" ");
  Serial.println(abs((int)(Va1*3)));

  if (Va1 > 0) {
    digitalWrite(7, LOW); //LOW is forward for Motor 1
    Serial.println("HIGH");
  }
  else if (Va1 < 0) {
    digitalWrite(7, HIGH); //LOW is forward for Motor 1
  }
  if (Va2 > 0) {
    digitalWrite(8, HIGH); //HIGH is forward for Motor 2
    Serial.println("HIGH");
  }
  else if (Va2 < 0){
    digitalWrite(8, LOW); //HIGH is forward for Motor 2
  }
  analogWrite(9,abs((int)Va1*20));
  analogWrite(10,abs((int)Va2*20));
  Ts = millis()-Tc;
  Tc = millis();
  
}

void encoderISRRIGHT(){
  unsigned long currentMillis = millis(); //tracks the current time in the program in ms. Used for debouncing.
  currentStateARIGHT = digitalRead(2); //reads in the current state of A/CLK.


  //if statement to check if the state of A has changed and time greater than the debounce time has passed.
  if ((currentStateARIGHT != previousStateARIGHT) && (currentMillis - previousMillisRIGHT >= debounceTime)) {
    
    currentStateBRIGHT = digitalRead(5); //reads in current state of B/DT.

    secIntervalRIGHT = (double)(currentMillis-previousMillisRIGHT)/((double)1000);
    if(secIntervalRIGHT != 0){
      if (secIntervalRIGHT > 100) {
        thetaDotRIGHT = 0;
      }
  
      //condition to check if the knob has turned CW or CWW. count up if CW, count down if CCW.
      if (currentStateARIGHT != currentStateBRIGHT) {
          //rotaryDirection = "CCW";
          counterRIGHT++;
          radianCountRIGHT = (((double)counterRIGHT/(double)countsPerRevolution)*2*pi);
  
          if (secIntervalRIGHT > 100) {
            thetaDotRIGHT = 0;
          }
          else {
            thetaDotRIGHT = (double)(radianCountRIGHT-preRadianCountRIGHT)/(double)(secIntervalRIGHT);
          }
        }
      else {
          //rotaryDirection = "CW";
          counterRIGHT--;
          radianCountRIGHT = (((double)counterRIGHT/(double)countsPerRevolution)*2*pi);
  
          if (secIntervalRIGHT > 100) {
            thetaDotRIGHT = 0;
          }
          else {
            thetaDotRIGHT = -1*((double)(radianCountRIGHT-preRadianCountRIGHT)/((double)(secIntervalRIGHT)));
          }
        }
    }
    }
    //store the time in the program where an output was produced. Used for debouncing.
    previousMillisRIGHT = currentMillis;
    preRadianCountRIGHT = radianCountRIGHT;
    previousStateARIGHT = currentStateARIGHT; //update previous state of A/CLK variable.
  }

void encoderISRLEFT(){
  unsigned long currentMillis = millis(); //tracks the current time in the program in ms. Used for debouncing.
  currentStateALEFT = digitalRead(3); //reads in the current state of A/CLK.
  secIntervalLEFT = (double)(currentMillis-previousMillisLEFT)/((double)1000);
  if(secIntervalLEFT != 0){
    if ((currentStateALEFT != previousStateALEFT) && (currentMillis - previousMillisLEFT >= debounceTime)) {
    currentStateBLEFT = digitalRead(6); //reads in current state of B/DT.
    
    //condition to check if the knob has turned CW or CWW. count up if CW, count down if CCW.
    if (currentStateALEFT != currentStateBLEFT) {
        //rotaryDirection = "CCW";
        counterLEFT++;
        radianCountLEFT = (((double)counterLEFT/(double)countsPerRevolution)*2*pi);
        if (secIntervalLEFT > 100) {
          thetaDotLEFT = 0;
        }
        else {
          thetaDotLEFT = (double)(radianCountLEFT-preRadianCountLEFT)/(double)(secIntervalLEFT);
        }
      }
    else {
        //rotaryDirection = "CW";
        counterLEFT--;
        radianCountLEFT = (((double)counterLEFT/(double)countsPerRevolution)*2*pi);
        if (secIntervalLEFT > 100) {
          thetaDotLEFT = 0;
        }
        else {
          thetaDotLEFT = -1*((double)(radianCountLEFT-preRadianCountLEFT)/((double)(secIntervalLEFT)));
        }
      }
      
    }
  }
    //store the time in the program where an output was produced. Used for debouncing.
    previousMillisLEFT = currentMillis;
    preRadianCountLEFT = radianCountLEFT;
    previousStateALEFT = currentStateALEFT; //update previous state of A/CLK variable.
  }

void turn(double angle){

  currentMillisAngle = millis();
  angleInterval = (float)(currentMillisAngle - previousMillisAngle);
    currentPhiDot = r*(thetaDotRIGHT-thetaDotLEFT)/d;
//    if (currentPhiDot > 20){
//      currentPhiDot = 20;
//    }
//    if (currentPhiDot < -20){
//      currentPhiDot = -20;
//    }
    currentPhi = currentPhi + (double)(angleInterval)*currentPhiDot;
    errorPhi = angle-currentPhi;
  previousMillisAngle = currentMillisAngle;

}
void rho(double distance){
  //currentDistance = errorRho*(millis()/1000);
  currentMillisDistance = millis();
  distanceInterval = (float)(currentMillisDistance - previousMillisDistance);
  double I = 0;

  currentRhoDot = r*(thetaDotRIGHT+thetaDotLEFT)/2;
//  if (currentRhoDot > 20){
//    currentRhoDot = 20;
//  }
//  if (currentRhoDot < -20){
//    currentRhoDot = -20;
//  }
//  if (isnan(currentRhoDot)){
//    currentRhoDot = 0;
//  }
  currentDistance += ((float)distanceInterval*(float)currentRhoDot)/2;

  errorRho = distance - currentDistance;

  double Kp = 4.2472;
  double Ki = 0.024677;

  I = I + TsRho*errorRho;

  deltaV = Kp*errorRho + Ki*I;

  
  

  TsRho = millis()-TcRho;
  TcRho = millis();
  
  previousMillisDistance = currentMillisDistance;
}
