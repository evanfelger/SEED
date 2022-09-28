//Code need to be modified to fit the project. (System input and output).

int counter = 0; //counter variable to track position of knob
int currentStateA; //tracks the current state of A/CLK
int previousStateA; //variable to store previous state of A/CLK
int currentStateB; //tracks current state of B/DT
String rotaryDirection; //string to store the rotation of the knob (CW or CCW). Will be used to print to the serial monitor.
void encoderISR(); //function prototype for the ISR.
unsigned long previousMillis = 0; //variable to track previous reading of the millis() function.
int debounceTime = 10; //debounce time. set to 10 ms.

double radianCount;
int countsPerRevolution = 64;

void setup() {

  pinMode(2, INPUT); //CLK or A
  pinMode(3, INPUT); //DT or B

  previousStateA = digitalRead(2); //reads in the current state of A/CLK to start off the program.

  attachInterrupt(0, encoderISR, CHANGE); //attach an interrupt to pin 2 (interrupt 0) to call the function encoderISR when any change happens in pin 2.
  
  
  
  pinMode(7, OUTPUT); //Motor Voltage Sign
  pinMode(9, OUTPUT); //Motor Voltage
  Serial.begin(9600);

  digitalWrite(7, LOW);
  
}

void loop() {
  for (int motorPWM = 0 ; motorPWM <= 255; motorPWM += 5) {
    
    analogWrite(9, motorPWM);
    
    delay(1000);
  }

  if (7 == LOW) {
    digitalWrite(7, HIGH);
  }

  else if (7 == HIGH) {
    digitalWrite(7, LOW);
  }
}

void encoderISR(){
  unsigned long currentMillis = millis(); //tracks the current time in the program in ms. Used for debouncing.
  currentStateA = digitalRead(2); //reads in the current state of A/CLK.

  //if statement to check if the state of A has changed and time greater than the debounce time has passed.
  if ((currentStateA != previousStateA) && (currentMillis - previousMillis >= debounceTime)) {
    
    currentStateB = digitalRead(3); //reads in current state of B/DT.

    //condition to check if the knob has turned CW or CWW. count up if CW, count down if CCW.
    if (currentStateA != currentStateB) {
        rotaryDirection = "CW";
        counter++;
      }
    else {
        rotaryDirection = "CCW";
        counter--;
      }

      //counts to radians
      radianCount = ((double)(counter/countsPerRevolution))*2*3.14159;
      
      //Prints out the direction and the relative position of the knob to when the program started.
      Serial.print(rotaryDirection);
      Serial.print(" ");
      Serial.print(radianCount);
      Serial.print("\n");

      //store the time in the program where an output was produced. Used for debouncing.
      previousMillis = currentMillis;

    
    }

    previousStateA = currentStateA; //update previous state of A/CLK variable.
  }
