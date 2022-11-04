//code from mini project VVVV (for pi arduino communication)
#include <Wire.h> 
#define SLAVE_ADDRESS 0x04 
int number0 = 0;
int number1 = 0;
int state = 0;
byte data[32] = {0};
byte storedata[32] = {0};
int arraylen = 0;

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

double r = 0.07; // Radius of the wheel
double d = .3993; // Distance Between Wheels

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
unsigned long currentMillisLEFT;
unsigned long currentMillisRIGHT;

void encoderISRLEFT(); //function prototype for the ISR.void encoderISR(); //function prototype for the ISR.
void encoderISRRIGHT();
void turn(double);
void rho(double);
double TsRho = 0;
double TcRho = millis();
double TsPhi = 0;
double TcPhi = millis();

double deltaV;
double deltaPhi;
  
double Kp = 6;//4.2472;
double Ki = 1;//0.024677;

double secIntervalLEFT;
double secIntervalRIGHT;
double IRho = 0;
double IPhi = 0;

int state1 = 1;
int state2 = 0;
int state3 = 0;


void setup() {
  
  currentDistance = 0;
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
  //digitalWrite(7, LOW); //LOW is forward for Motor 1
  //digitalWrite(8, HIGH); //HIGH is forward for Motor 2
  pinMode(13, OUTPUT);
  Serial.begin(9600);


  attachInterrupt(1, encoderISRLEFT, CHANGE); 
  attachInterrupt(0, encoderISRRIGHT, CHANGE); //attach an interrupt to pin 2 (interrupt 0) to call the function encoderISR when any change happens in pin 2.
  state1 = 0;
  state2 = 1;
}

void loop() {

  Wire.begin(SLAVE_ADDRESS);
    // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  
  timeNow = millis();
  if(state1 == 1){
    targetAngle = pi;//radians
    targetDistance = 60.96; //cm
    rho(targetDistance);
    turn(targetAngle);
    if(round(errorRho*1000) == 0){
      state1 = 0;
      state2 = 0;
      state3 = 1;
    }
    
  }
  if(state2 == 1){
    targetAngle = pi;//degrees
    deltaV = 0;
    //targetDistance = 76.2;//mm
    //rho(targetDistance);
    turn(targetAngle);
    if(round(errorPhi*10000) == 0){
      state1 = 1;
      state2 = 0;
      state3 = 0;
    }
    
  }

  if(state3 == 1){
    deltaV = 0;
    deltaPhi = 0;
  }
  
  Va1 = (double)((deltaV + deltaPhi)/2);
  Va2 = (double)((deltaV - deltaPhi)/2);
  if (Va1 > 0) {
    digitalWrite(7, LOW); //LOW is forward for Motor 1
  }
  else if (Va1 < 0) {
    digitalWrite(7, HIGH); //LOW is forward for Motor 1
  }
  if (Va2 > 0) {
    digitalWrite(8, HIGH); //HIGH is forward for Motor 2
  }
  else if (Va2 < 0){
    digitalWrite(8, LOW); //HIGH is forward for Motor 2
  }
  if (abs(Va1) > 80){
    Va1 = 80;
  }
  if (abs(Va2) > 80){
    Va2 = 82;
  }
  
  analogWrite(9,abs(round(Va1)));
  analogWrite(10,abs(round(Va2)));
}

void encoderISRRIGHT(){
  currentMillisRIGHT = millis(); //tracks the current time in the program in ms. Used for debouncing.
  currentStateARIGHT = digitalRead(2); //reads in the current state of A/CLK.


  //if statement to check if the state of A has changed and time greater than the debounce time has passed.
  if ((currentStateARIGHT != previousStateARIGHT) && (currentMillisRIGHT - previousMillisRIGHT >= debounceTime)) {
    
    currentStateBRIGHT = digitalRead(5); //reads in current state of B/DT.

    secIntervalRIGHT = (double)(currentMillisRIGHT-previousMillisRIGHT)*((double)(.001));
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
            thetaDotRIGHT = (double)((double)(radianCountRIGHT-preRadianCountRIGHT)/((double)(secIntervalRIGHT)));
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
            thetaDotRIGHT = (double)((double)(radianCountRIGHT-preRadianCountRIGHT)/((double)(secIntervalRIGHT)));
          }
        }
    }
    }
    //store the time in the program where an output was produced. Used for debouncing.
    previousMillisRIGHT = currentMillisRIGHT;
    preRadianCountRIGHT = radianCountRIGHT;
    previousStateARIGHT = currentStateARIGHT; //update previous state of A/CLK variable.
  }

void encoderISRLEFT(){
  currentMillisLEFT = millis(); //tracks the current time in the program in ms. Used for debouncing.
  currentStateALEFT = digitalRead(3); //reads in the current state of A/CLK.
  secIntervalLEFT = (double)(currentMillisLEFT-previousMillisLEFT)*((double)(.001));
  if(secIntervalLEFT != 0){
    if ((currentStateALEFT != previousStateALEFT) && (currentMillisLEFT - previousMillisLEFT >= debounceTime)) {
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
          thetaDotLEFT = -1*(double)(((double)(radianCountLEFT-preRadianCountLEFT))/((double)(secIntervalLEFT)));
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
          thetaDotLEFT = -1*(double)(((double)(radianCountLEFT-preRadianCountLEFT))/((double)(secIntervalLEFT))); //radians/sec
        }
      }
      
    }
  }
    //store the time in the program where an output was produced. Used for debouncing.
    previousMillisLEFT = currentMillisLEFT;
    preRadianCountLEFT = radianCountLEFT;
    previousStateALEFT = currentStateALEFT; //update previous state of A/CLK variable.
  }

void turn(double angle){
  Kp = 2000;
  Ki = 5;
  currentMillisAngle = millis();
  angleInterval = (float)(currentMillisAngle - previousMillisAngle)*((double)(.001));
  currentPhiDot = r*(thetaDotRIGHT-thetaDotLEFT)/d; // Radians/sec
  currentPhi += (double)((double)(angleInterval)*(double)currentPhiDot); // radians
  errorPhi = (angle-currentPhi);//multiplier added (should be in radian magnitudes, 
  if((round(errorPhi*10)==0.00)){
    IPhi = 0;
  }
  IPhi = IPhi + TsPhi*errorPhi;
  deltaPhi = Kp*errorPhi + Ki*IPhi;
  TsPhi = millis()-TcPhi;
  TcPhi = millis();
  previousMillisAngle = currentMillisAngle;

}
void rho(double distance){
  Kp = 10;
  Ki = 2;
  //currentDistance = errorRho*(millis()/1000);
  currentMillisDistance = millis();
  distanceInterval = (float)(currentMillisDistance - previousMillisDistance)/10;
  currentRhoDot = r*(thetaDotRIGHT+thetaDotLEFT)/2;
  currentDistance += (double)((double)distanceInterval*(double)currentRhoDot);
  errorRho = distance - currentDistance;
  if((round(errorRho*10)==0.00)){
    IRho = 0;
  }
  IRho = IRho + TsRho*errorRho;
  deltaV = Kp*errorRho + Ki*IRho;
  TsRho = millis()-TcRho;
  TcRho = millis();
  previousMillisDistance = currentMillisDistance;
}

void receiveData(int byteCount){
  int i = 0;
  if (Wire.available() != 0){
    while(Wire.available()) {
      data[i] = Wire.read();
      data[i] = (double)data[i]/(double)100; 
      Serial.print(data[i]); 
      Serial.print(' ');
      i++;
    
      arraylen++; 
    }
  }
}
