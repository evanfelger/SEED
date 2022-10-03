//Code need to be modified to fit the project. (System input and output).
//#include <cmath>
#include <Wire.h> 
#define SLAVE_ADDRESS 0x04 
int number0 = 0;
int number1 = 0;
int state = 0;
byte data[32] = {0};
byte storedata[32] = {0};
int arraylen = 0;


int counter = 0; //counter variable to track position of knob
int currentStateA; //tracks the current state of A/CLK
int previousStateA; //variable to store previous state of A/CLK
int currentStateB; //tracks current state of B/DT
String rotaryDirection; //string to store the rotation of the knob (CW or CCW). Will be used to print to the serial monitor.
void encoderISR(); //function prototype for the ISR.
unsigned long previousMillis = 0; //variable to track previous reading of the millis() function.
int debounceTime = 0; //debounce time. set to 4 ms.

double radianCount;
int countsPerRevolution = 1600; //320
int motorPWM;

double radianCount2; //radians
double Kp;// = 5;//2.2472; // porportion
double Ki;// = .84677; // integral number
double I; 
double e_past; //last error
double Ts; 
double Tc;
double e;
double r;
double y;
double d;
double u;
double pi = 3.14159;
double mod;
int currentCount;
int timeNow;
//int arraylen = 0; 
double currentRadian;

void setup() {




  I = 0;
  e_past = 0;
  Ts = 0;
  Tc = millis();
  //counts to radians




  

  pinMode(2, INPUT); //CLK or A
  pinMode(3, INPUT); //DT or B

  previousStateA = digitalRead(2); //reads in the current state of A/CLK to start off the program.

  attachInterrupt(0, encoderISR, CHANGE); //attach an interrupt to pin 2 (interrupt 0) to call the function encoderISR when any change happens in pin 2.
  
  
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  pinMode(7, OUTPUT); //Motor Voltage Sign
  pinMode(9, OUTPUT); //Motor Voltage
  Serial.begin(250000);

  digitalWrite(7, LOW);

  pinMode(13, OUTPUT);
  
  
  //Serial.println("Ready!");

  
}

void loop() {
    timeNow = millis();
    currentCount = counter;

    Wire.begin(SLAVE_ADDRESS);
    // define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);
    //oldData = receiveData;
    //for (int n =0; n < sizeof(data);n++){
//      for(int i=0; i<arraylen; i++){
//          //Serial.print(data[1]);
//          
// }
 //Serial.println(data[1]);
          if (data[1] == byte(1)){
            r = 0;
          }
          else if (data[1] == byte(2)){
            r = pi/2;
          }
          else if (data[1] == byte(3)){
            r = pi;
          }
          else if (data[1] == byte(4)){
            r = 3*pi/2;
          }
          else{
            r = r;
          }
    //Serial.println();
    arraylen = 0;
    //}
  //delay(100);
  currentCount = counter;
  currentRadian = radianCount;
  
  

  
  Kp = 4.2472;//2.2472; // porportion
  Ki = 0.024677;//.24677; // integral number .44677
  //r = 5*pi/2; //target in radians
  e = radianCount-r; //error in radians
  //Serial.println("error");
  //Serial.println(180/pi*e);
  if((round(e*100)==0.00)){
    I = 0;
    //Serial.println("HERE");
  }
  if (Ts>0){
    e_past = e;
  }
  else{
    d = 0;
  }
  I = I + Ts*e;
  u = Kp*e + Ki*I; //Voltage output, can be negative
  //Serial.println("e");
  //Serial.println(e);
  //Serial.println(u);
  if (u<0){
    digitalWrite(7, HIGH);
  }
  else{
    digitalWrite(7, LOW);
  }
  if (abs(u)<.5 && u != 0){
    u = 1;
  }
  if (abs(u)>255){
    u = 255;
  }
  analogWrite(9, abs(u));
  Serial.println(r);
  Ts = millis()-Tc;
  Tc = millis();
  
  //for (int motorPWM = 0 ; motorPWM <= 255; motorPWM += 5) {
    
    //analogWrite(9, motorPWM);
    
    //delay(1000);
  //}

  //if (digitalRead(7) == LOW) {
    //digitalWrite(7, HIGH);
  //}
  
//  if (radianCount == 0 ){
//    motorPWM = 100;
//    analogWrite(9, motorPWM);
//  }
//  //digitalWrite(7, HIGH);
//  if (radianCount2 == 3.14){
//    digitalWrite(7, LOW);
//    analogWrite(9, 0);
//    motorPWM = 0;
    
    
  //}
  //else if (digitalRead(7) == HIGH) {
   // digitalWrite(7, LOW);
  //}
  
  //while(millis() < timeNow + 15){
        //wait approx. 15 ms
    //}
  Serial.print("velocity: ");
  Serial.print((radianCount - currentRadian)/.015);
  Serial.println(" rad/s");
}

void printHex(uint8_t num) {
  char hexCar[2];

  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}

void receiveData(int byteCount){
  int i = 0;
  if (Wire.available() != 0){
    while(Wire.available()) {
    data[i] = Wire.read(); 
    //Serial.print(data[i]); 
    //Serial.print(' ');
    i++;
    
    arraylen++; 
  }
  }
  
  //i--;
  //Serial.println(' ');
  //int j = 0;
//  for (int k = i - 1; k != -1 ; k--){
//  
//    storedata[j] = data[k+1];
//    
//    j++; 
//  }
}

void sendData(){
  Wire.write(storedata , 32);
}


void encoderISR(){
  unsigned long currentMillis = millis(); //tracks the current time in the program in ms. Used for debouncing.
  currentStateA = digitalRead(2); //reads in the current state of A/CLK.

  //if statement to check if the state of A has changed and time greater than the debounce time has passed.
  if ((currentStateA != previousStateA) && (currentMillis - previousMillis >= debounceTime)) {
    
    currentStateB = digitalRead(3); //reads in current state of B/DT.

    //condition to check if the knob has turned CW or CWW. count up if CW, count down if CCW.
    if (currentStateA != currentStateB) {
        //rotaryDirection = "CCW";
        counter--;
      }
    else {
        //rotaryDirection = "CW";
        counter++;
      }
      
      radianCount = (((double)counter/(double)countsPerRevolution)*2*pi);
      //radianCount2 = round(radianCount*((Kp/2*s+Ki/2)/(s^2+(4+Kp)/2*s+Ki/2)*100.00)/100.00);
      //Prints out the direction and the relative position of the knob to when the program started.
      //Serial.print(rotaryDirection);
      //Serial.print(" ");
      //Serial.print(counter);
      //Serial.print("\n");

      //store the time in the program where an output was produced. Used for debouncing.
      previousMillis = currentMillis;
      
    
    }

    previousStateA = currentStateA; //update previous state of A/CLK variable.
  }
