//code from mini project VVVV (for pi arduino communication)
#include <Wire.h> 
#define SLAVE_ADDRESS 0x04 
int number0 = 0;
int number1 = 0;
int state = 0;
byte data[32] = {'q'};
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
unsigned long pastinterval;
unsigned long interval;

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
int state4 = 0;

int marker;
double pastdistance;

//for system integration
int angleFound = 0;
double angleInput;
int distanceFound = 0;
double distanceInput;

void receiveData(int);


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
  state1 = 1;
  state2 = 0;
  angleFound = 0;
  //targetAngle = pi/4;
  data[0] = 'q';
  delay(1000);
}

void loop() {

  Wire.begin(SLAVE_ADDRESS);
    // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  
  
  timeNow = millis();


  
  if(state1 == 1){
    delay(100);
    if(data[0] != 'q'){
      state1 = 0;
      state2 = 1;
      state3 = 0;
    }
    //Serial.print("State 1");
    //degrees
    deltaV = 0;
    turn(targetAngle);
    drive();
    if(round(errorPhi*10) == 0.00) {
      
      //Serial.println(data[0]);
      if(data[0] != 'q'){
        state1 = 0;
        state2 = 1;
        state3 = 0;
        //currentPhi = 0;
        if (data[1] == 0) {
          angleInput = data[2]*0.01 + data[3]*0.0001;
        }
        else if (data[1] == 1) {
          angleInput = -1*data[2]*0.01 + data[3]*0.0001;
        }
      }
      else{
        targetAngle = targetAngle+(pi/8);
      }
      
    } 
  }
  



  if(state2 == 1){
    pastinterval = millis();
    //Serial.print("State 2");
    deltaV = 0;
    //targetDistance = 76.2;//mm
    //rho(targetDistance);
    if (data[1] == 0) {
          angleInput = data[2]*0.01 + data[3]*0.0001;
        }
        else if (data[1] == 1) {
          angleInput = -1*data[2]*0.01 + data[3]*0.0001;
        }
    turn(targetAngle - angleInput);
    drive();
    //Serial.print(targetAngle - angleInput);
    //Serial.print(" ");
    //Serial.println(currentPhi);
    if((round(errorPhi*1000) == 0.000) and (interval > 1000)){
      state1 = 0;
      state2 = 0;
      state3 = 1;  
    }
    interval += millis()-pastinterval;
  }


  
  if(state3 == 1){
    //Serial.print("State 3");
    //targetAngle = pi;//radians
    //if (distanceFound == 0) {
      //distanceInput = data[4];
      //distanceFound = 1;
     // }
     if (data[1] == 0) {
          angleInput = data[2]*0.01 + data[3]*0.0001;
        }
        else if (data[1] == 1) {
          angleInput = -1*data[2]*0.01 + data[3]*0.0001;
        }
    turn(targetAngle - angleInput);
    rho(data[4]);
    drive();
    //turn(targetAngle);
    Serial.print(data[4]);
    Serial.print(" ");
    Serial.print(errorRho);
    Serial.print(" ");
    Serial.println(currentDistance);
    
    if(round(errorRho*1000) == 0){
      state1 = 0;
      state2 = 0;
      state3 = 0;
      state4 = 1;
    }
    
  }


  if(state4 == 1){
    //Serial.print("State 4");
    deltaV = 0;
    deltaPhi = 0;
  }


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
  Kp = 1000;
  Ki = 0;
  currentMillisAngle = millis();
  angleInterval = (float)(currentMillisAngle - previousMillisAngle)*((double)(.001));
  currentPhiDot = r*(thetaDotRIGHT-thetaDotLEFT)/d; // Radians/sec
  currentPhi += (double)((double)(angleInterval)*(double)currentPhiDot); // radians
  errorPhi = (angle-currentPhi);//multiplier added (should be in radian magnitudes, 
  if((round(errorPhi*100000)==0.00)){
    IPhi = 0;
  }
  IPhi = IPhi + TsPhi*errorPhi;
  deltaPhi = Kp*errorPhi + Ki*IPhi;
  TsPhi = millis()-TcPhi;
  TcPhi = millis();
  previousMillisAngle = currentMillisAngle;

}
void rho(double distance){
  Kp = 5;
  Ki = 0;
  //currentDistance = errorRho*(millis()/1000);
  currentMillisDistance = millis();
  distanceInterval = (float)(currentMillisDistance - previousMillisDistance)/10;
  currentRhoDot = r*(thetaDotRIGHT+thetaDotLEFT)/2;
  currentDistance += (double)((double)distanceInterval*(double)currentRhoDot);
  if(round(distance) == round(pastdistance)){
    if(marker == 1){
      Serial.println("here");
      currentDistance = 0;
    }
    errorRho = distance - currentDistance;
    marker = 0;
  }
  else{
    errorRho = distance;// - currentDistance;
    marker = 1;
  }
  //errorRho = distance - currentDistance;
  if((round(errorRho*10)==0.00)){
    IRho = 0;
  }
  IRho = IRho + TsRho*errorRho;
  deltaV = Kp*errorRho + Ki*IRho;
  TsRho = millis()-TcRho;
  TcRho = millis();
  previousMillisDistance = currentMillisDistance;
  pastdistance = distance;
}

void receiveData(int byteCount){
  int i = 0;
  if ((Wire.available() != 0) && (i < 5)){
    while(Wire.available()) {
      data[i] = Wire.read();
      //data[i] = (double)data[i]/(double)100; 
//      Serial.print(i);
//      Serial.print(" ");
//      Serial.print(data[i]); 
//      Serial.print(' ');
      i++;
    
      arraylen++; 
    }
  }
  //Serial.print("\n");
}


void drive() {
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
  if (abs(Va1) > 100){
    Va1 = 100;
  }
  if (abs(Va2) > 100){
    Va2 = 100;
  }
  
  analogWrite(9,abs(round(Va1)));
  analogWrite(10,abs(round(Va2)));
  }
