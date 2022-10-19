//Varaibles for motor 1
int counter1 = 0; //counter variable to track position of knob
int counter2 = 0; //counter variable to track position of knob
int currentState1A; //tracks the current state of A/CLK
int previousState1A; //variable to store previous state of A/CLK
int currentState2A; //tracks the current state of A/CLK
int previousState2A; //variable to store previous state of A/CLK
int currentState1B; //tracks current state of B/DT
int currentState2B; //tracks current state of B/DT
double radianCount1;
double radianCount2;
double pi = 3.14159;
int currentCount1;
int currentCount2;
int timeNow;
double currentRadian1;
double currentRadian2;
int i = 0;
double r = 0.075; // Radius of the wheel
double d = .36; // Distance Between Wheels
double preRho;
double prePhi;
double Va1;
double Va2;
double Va;
double deltaVa;
int debounceTime = 0; //debounce time. set to 4 ms.
int countsPerRevolution = 1600; //320
int motorPWM;
double targetAngle;
double targetDistance;
double targetPhi, targetRho;
double errorPhi;
double errorRho;
double theta1, theta2, preRadianCount1, preRadianCount2, pastMillis1, pastMillis2;
double currentRadianCount1, currentRadianCount2;
double currentPhi, currentRho;
double currentDistance = 0;



unsigned long previousMillis1 = 0; //variable to track previous reading of the millis() function.
unsigned long previousMillis2 = 0;
unsigned long currentMillis1 = 0;
unsigned long currentMillis2 = 0;
unsigned long prevMillis = 0;

void encoderISRleft(); //function prototype for the ISR.void encoderISR(); //function prototype for the ISR.
void encoderISRright();
void phi(double angle);
void rho(double distance);

void setup() {
  // put your setup code here, to run once:
  pinMode(2, INPUT); //motor 1 CLK or A
  pinMode(5, INPUT); //motor 1 DT or B
  pinMode(3, INPUT); //motor 2 CLK or A
  pinMode(6, INPUT); //motor 2 DT or B
  previousState1A = digitalRead(2); //motor 1 reads in the current state of A/CLK to start off the program.
  previousState2A = digitalRead(5); //motor 2 reads in the current state of A/CLK to start off the program.
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  pinMode(8, OUTPUT); //Motor 1 Voltage Sign
  pinMode(9, OUTPUT); //Motor 1 Voltage
  pinMode(7, OUTPUT); //Motor 2 Voltage Sign
  pinMode(10, OUTPUT); //Motor 2 Voltage
  digitalWrite(7, HIGH);
  digitalWrite(8, LOW);
  pinMode(13, OUTPUT);
  Serial.begin(250000);


  attachInterrupt(1, encoderISRleft, CHANGE); 
  attachInterrupt(0, encoderISRright, CHANGE); //attach an interrupt to pin 2 (interrupt 0) to call the function encoderISR when any change happens in pin 2.
}

void loop() {
  timeNow = millis();
  currentCount1 = counter1;
  currentRadian1 = radianCount1;
  targetAngle = 0;
  targetDistance = 10;
  rho(targetDistance);
  phi(targetAngle);
  Va1 = (errorRho + errorPhi)/2;
  Va2 = (errorRho - errorPhi)/2;
  digitalWrite(9,Va1);
  digitalWrite(10,Va2);
  
}

void encoderISRright(){
  unsigned long currentMillis1 = millis(); //tracks the current time in the program in ms. Used for debouncing.
  currentState1A = digitalRead(2); //reads in the current state of A/CLK.


  //if statement to check if the state of A has changed and time greater than the debounce time has passed.
  if ((currentState1A != previousState1A) && (currentMillis1 - previousMillis1 >= debounceTime)) {
    
    currentState1B = digitalRead(5); //reads in current state of B/DT.

    //condition to check if the knob has turned CW or CWW. count up if CW, count down if CCW.
    if (currentState1A != currentState1B) {
        //rotaryDirection = "CCW";
        counter1--;
      }
    else {
        //rotaryDirection = "CW";
        counter1++;
      }
      
      radianCount1 = (((double)counter1/(double)countsPerRevolution)*2*pi);
      theta1 = (radianCount1-preRadianCount1)/(millis()-pastMillis1);
      pastMillis1 = millis();
      preRadianCount1 = currentRadianCount1;
    }
    //store the time in the program where an output was produced. Used for debouncing.
    previousMillis1 = currentMillis1;
    previousState1A = currentState1A; //update previous state of A/CLK variable.
  }

void encoderISRleft(){
  unsigned long currentMillis2 = millis(); //tracks the current time in the program in ms. Used for debouncing.
  currentState2A = digitalRead(3); //reads in the current state of A/CLK.

    if ((currentState2A != previousState2A) && (currentMillis2 - previousMillis2 >= debounceTime)) {
    
    currentState2B = digitalRead(6); //reads in current state of B/DT.

    //condition to check if the knob has turned CW or CWW. count up if CW, count down if CCW.
    if (currentState2A != currentState2B) {
        //rotaryDirection = "CCW";
        counter2--;
      }
    else {
        //rotaryDirection = "CW";
        counter2++;
      }
      
      radianCount2 = (((double)counter2/(double)countsPerRevolution)*2*pi);
      theta2 = (radianCount2-preRadianCount2)/(millis()-pastMillis2);
      pastMillis2 = millis();
      preRadianCount2 = currentRadianCount2;
    }
    //store the time in the program where an output was produced. Used for debouncing.
    previousMillis2 = currentMillis2;
    previousState2A = currentState2A; //update previous state of A/CLK variable.
  }

void phi(double angle){
  targetPhi = angle;
  currentPhi = r*(theta1-theta2)/d;
  errorPhi = currentPhi - targetPhi;
  if (abs(errorPhi) < .1){
    errorPhi = 0;
  }

}
void rho(double distance){
  currentDistance = errorRho*(millis()/1000);
  Serial.print(distance);
  Serial.print(" ");
  Serial.println(currentDistance);
  if (distance >= currentDistance){
    targetRho = 5;
    currentRho = r*(theta1+theta2)/2;
    errorRho = targetRho - currentRho;
  }
  else{
    errorRho = 0;
  }
  prevMillis = millis();
}
