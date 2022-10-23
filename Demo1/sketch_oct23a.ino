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

unsigned long previousMillisRIGHT = 0; //variable to track previous reading of the millis() function.
unsigned long previousMillisLEFT = 0;
unsigned long prevMillis = 0;

unsigned long previousMillisAngle;
unsigned long previousMillisDistance;

void encoderISRLEFT(); //function prototype for the ISR.void encoderISR(); //function prototype for the ISR.
void encoderISRRIGHT();
void turn(double);
void rho(double);
double TsRho = 0;
double TcRho = millis();

double deltaV;


void setup() {
  // put your setup code here, to run once:
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
  digitalWrite(7, HIGH);
  digitalWrite(8, LOW);
  pinMode(13, OUTPUT);
  Serial.begin(250000);


  attachInterrupt(1, encoderISRLEFT, CHANGE); 
  attachInterrupt(0, encoderISRRIGHT, CHANGE); //attach an interrupt to pin 2 (interrupt 0) to call the function encoderISR when any change happens in pin 2.
}

void loop() {
  timeNow = millis();
  currentCountRIGHT = counterRIGHT;
  currentRadianRIGHT = radianCountRIGHT;
  targetAngle = pi/2;
  targetDistance = 0;
  rho(targetDistance);
  turn(targetAngle);
  Va1 = (errorRho + errorPhi)/2;
  Va2 = (errorRho - errorPhi)/2;
  analogWrite(9,Va1);
  analogWrite(10,Va2);
  
}

void encoderISRRIGHT(){
  unsigned long currentMillis = millis(); //tracks the current time in the program in ms. Used for debouncing.
  currentStateARIGHT = digitalRead(2); //reads in the current state of A/CLK.


  //if statement to check if the state of A has changed and time greater than the debounce time has passed.
  if ((currentStateARIGHT != previousStateARIGHT) && (currentMillis - previousMillisRIGHT >= debounceTime)) {
    
    currentStateBRIGHT = digitalRead(5); //reads in current state of B/DT.

    double secIntervalRIGHT = (double)(currentMillis-previousMillisRIGHT)/1000;

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
          thetaDotRIGHT = (radianCountRIGHT-preRadianCountRIGHT)/(secIntervalRIGHT);
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
          thetaDotRIGHT = -1*((radianCountRIGHT-preRadianCountRIGHT)/(secIntervalRIGHT));
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

  double secIntervalLEFT = (double)(currentMillis-previousMillisLEFT)/1000;

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
          thetaDotLEFT = (radianCountLEFT-preRadianCountLEFT)/(secIntervalLEFT);
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
          thetaDotLEFT = -1*((radianCountLEFT-preRadianCountLEFT)/(secIntervalLEFT));
        }
      }
      
    }
    //store the time in the program where an output was produced. Used for debouncing.
    previousMillisLEFT = currentMillis;
    preRadianCountLEFT = radianCountLEFT;
    previousStateALEFT = currentStateALEFT; //update previous state of A/CLK variable.
  }

void turn(double angle){

  unsigned long currentMillisAngle = millis();
  double angleInterval = (double)(currentMillisAngle - previousMillisAngle)/1000;
  
  currentPhiDot = r*(thetaDotRIGHT-thetaDotLEFT)/d;
  currentPhi = currentPhi + angleInterval*currentPhiDot;
  errorPhi = currentPhi - angle;
  if (abs(errorPhi) < .1){
    errorPhi = 0;
  }

  previousMillisAngle = currentMillisAngle;

}
void rho(double distance){
  //currentDistance = errorRho*(millis()/1000);
  unsigned long currentMillisDistance = millis();
  double distanceInterval = (double)(currentMillisDistance - previousMillisDistance)/1000;
  double I = 0;

  currentRhoDot = r*(thetaDotRIGHT+thetaDotLEFT)/2;
  currentDistance = currentDistance + distanceInterval*currentRhoDot;

  errorRho = distance - currentDistance;

  double Kp = 4.2472;
  double Ki = 0.024677;

  I = I + TsRho*errorRho;

  deltaV = Kp*errorRho + Ki*I;

  if (deltaV<0){
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
  }
  else{
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
  }

  TsRho = millis()-TcRho;
  TcRho = millis();
  
 
  //if (distance >= currentDistance){
    //targetRho = 5;
    //currentRho = r*(thetaDotRIGHT+thetaDotLEFT)/2;
    //errorRho = targetRho - currentRho;
  //}
  //else{
    //errorRho = 0;
  //}
  previousMillisDistance = currentMillisDistance;
}
