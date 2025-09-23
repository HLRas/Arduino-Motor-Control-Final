#include <Arduino.h>
// =====================================================================//
// Function prototypes
void runMeasureComms();
void PID(float dt_s);
void runUltras();
void pinSetup();
void printSpeeds();
void applyPWMs(bool constrain_pwm = true);
void runComms();
void calcSpeeds();
void riseL();
void riseR();
void adjustDirection();
void stopMotors(int whichmotor = 2);
void forwardL();
void reverseL();
void forwardR();
void reverseR();

// =====================================================================//
// Reference to where motor is, look from back to front of car

// Right motor pins
const uint8_t enAR = 9;
const uint8_t in1R = 8;
const uint8_t in2R = 7;
const uint8_t speedPinR = 3;

// Left motor pins
const uint8_t enBL = 11;
const uint8_t in3L = 12;
const uint8_t in4L = 13;
const uint8_t speedPinL = 2;

// =====================================================================//
// Ultrasonic sensors
const uint8_t trigUltraPin = 14; // Trigger pin
const uint8_t echoUltraPin[4] = {15,16,17,18} ;// Front, Right, Back, Left

// =====================================================================//
// Constants for measuring speed
#define GAPS 30.0 // Amount of slots on encoder wheel
float TS = 40.0; // System sampling time (ms)
#define MIN_PWM 40 // Helps reduce integral windup
#define MAX_PWM 255 // Max allowed pwm
#define WAIT_RISES 1 // # rising edges until new speed can be measured (excluding first) 

// =====================================================================//
// Global speed measurements values
float speedL = 0;
float speedR = 0; // m/s

// =====================================================================//
// Global speed requirements
volatile float desiredSpeedL = 0.4; // m/s
volatile float desiredSpeedR = 0.4; // m/s

// =====================================================================//
// System control variables

// PWM inputs
float pwmL = 70; // Start at higher value to prevent windup
float pwmR = 70; // Start at higher value to prevent windup

// PID parameters
float KpL = 35, KiL = 0, KdL = 8;
float KpR = 35, KiR = 0, KdR = 8;

// PID variables
float errorL = 0, errorR = 0;
float derivL = 0, derivR = 0;
float integralL = 0, integralR = 0;
float prevErrorL = 0, prevErrorR = 0;

// =====================================================================//
// Speed measurement times
volatile unsigned long dtL = 0 ; // us
volatile unsigned long dtR = 0 ; // us
volatile unsigned long lastMeasureL_us = 0; //us
volatile unsigned long lastMeasureR_us = 0; //us
unsigned long measureTimeout_us = 100000; // If no new interrupts, reset speed to 0 after this time

// =====================================================================//
// Speed measurement coeeficient
const float radius = 0.0099; // m
const float speed_coeff = (radius)*(M_PI/GAPS*WAIT_RISES)*(10e6);
                        //(radius)*(dtheta)*(u_sec/s) 

// =====================================================================//
// Timing variables
unsigned long currentTime_ms = millis();
unsigned long startTime_ms ;
unsigned long currentTime_us = micros();
// =====================================================================//
// Tester variables
bool closedLoop = true; // Should the system run the closedloop system?
bool echoSpeeds = false; // Should the system echo the speeds to terminal?
bool ultraEnabled = true; // Should we run ultrasonic measurements?
bool ultraStopEnabled = true; // Should the car stop if dist < threshold?
bool commsEnabled = false; // Should the arduino send data to the Jetson?
// =====================================================================//
// Ultrasonic measurements
float ultraDist[4] = {100,100,100,100}; // Distances in meters
float stopThres[4] = {20,10,10,10}; // cm
float ultraStops[4] = {0, 0 ,0 ,0}; // Flag which will stop the motor if too close
float ultraStop = 0;
// =====================================================================// 
// Main setup
// =====================================================================//           
void setup(){
  Serial.begin(115200); // Start serial communications
  while(!Serial); // Wait for serial connection to establish

  startTime_ms = millis(); // Software starts here

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(speedPinL), riseL, RISING);
  attachInterrupt(digitalPinToInterrupt(speedPinR), riseR, RISING);

  // Set up pins
  pinSetup();

  stopMotors(); // Turn off motors - Initial state
  adjustDirection(); // Sets the motors in correct direction
  applyPWMs(); // Apply specified pwm signals with constraints

  if (echoSpeeds){
    runMeasureComms();
  }
}

// =====================================================================// 
// Main loop
// =====================================================================// 
void loop(){
  static int dt_ms = 0;
  static float dt_s = 0;
  static unsigned long prevTime_ms = 0;
  currentTime_ms = millis();

  if (commsEnabled){ // Run communication to Jetson
    runComms();
  }

  if (ultraEnabled){ // Run ultrasonic sensors
    runUltras();
  }

  if (!ultraStop){
    if (closedLoop){
      currentTime_us = micros();
      if (currentTime_us - lastMeasureL_us > measureTimeout_us){
        dtL = 0; // speed calc handles 0 dt as 0 speed
        speedL = 0;
      }
      if (currentTime_us - lastMeasureR_us > measureTimeout_us){
        dtR = 0;
        speedR = 0;
      }

      dt_ms = currentTime_ms - prevTime_ms ; // Update elapsed time
      if (dt_ms > TS){ // if elapsed time > sampling time

        prevTime_ms = currentTime_ms;
        dt_s = dt_ms / 1000.0; 

        calcSpeeds(); // Update speeds

        PID(dt_s); // Run PID controller  

        applyPWMs(); // Apply PWMs

      }
    }else{
      applyPWMs(false); // Apply pwms without constraints
    }
  }else{
    stopMotors();
  }

  if (echoSpeeds){
    printSpeeds();
  }
  
}

// =====================================================================// 
// PID controller
// =====================================================================// 
void PID(float dt_s){
  // PID Control mode
  // Left wheel PID
  errorL = desiredSpeedL - speedL;
  integralL += errorL * (dt_s);
  derivL = (errorL - prevErrorL) / (dt_s);
  pwmL += KpL * errorL + KiL * integralL + KdL * derivL;
  prevErrorL = errorL;

  // Right wheel PID
  errorR = desiredSpeedR - speedR;
  integralR += errorR * (dt_s);
  derivR = (errorR - prevErrorR) / (dt_s);
  pwmR += KpR * errorR + KiR * integralR + KdR * derivR;
  prevErrorR = errorR;
}
// =====================================================================// 
// Comms for measurments
// =====================================================================// 
void runMeasureComms(){
  while(!Serial.available());
  if (Serial.available() > 0) {
      
      String msg = Serial.readStringUntil('\n');
      Serial.print("got message: ");
      Serial.print(msg);
      Serial.print(" at: ");
      Serial.println(millis());
      msg.trim(); // Remove any whitespace/newline characters
      
      float params[7]; 

      int commaIndex = msg.indexOf(',');
      int prevIndex = 0;
      int i = 0;
      // Find the comma delimiter - ADD BOUNDS CHECK HERE
      while (commaIndex != -1 && i < 7){
        String param = msg.substring(prevIndex, commaIndex);
        params[i++] = param.toFloat();
        prevIndex = commaIndex+1;
        commaIndex = msg.indexOf(',', prevIndex);
      }
      // Handle the last item - ADD BOUNDS CHECK HERE
      if (prevIndex < static_cast<int>(msg.length()) && i < 7) {
          params[i++] = msg.substring(prevIndex).toFloat();
      }

      // Assign each param
      KpL = params[0];
      KiL = params[1];
      KdL = params[2];
      KpR = params[3];
      KiR = params[4];
      KdR = params[5];
      TS  = params[6]; 
      Serial.println("Received args!!");
    }
}

// =====================================================================// 
// Pin setup
// =====================================================================// 
void runUltras(){
  static int duration = 0;

  for (int i = 0; i<4; i++){
    digitalWrite(trigUltraPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigUltraPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigUltraPin, LOW);

    //pinMode(echoUltraPin[i], INPUT);
    duration = pulseIn(echoUltraPin[i], HIGH);
    
    // Convert the time into a distance
    ultraDist[i] = (duration/2) / 29.1;

    // Stop car if needed and allowed
    if (ultraStopEnabled){
      if (ultraDist[i] < stopThres[i]){
        ultraStops[i] = 1;
      }else{
        ultraStops[i] = 0;
      }
    }else{
      ultraStop = 0;
    }
    
    //Serial.println(ultraStop);
    delay(1); // might need this for it to work
  }

  ultraStop = 0;
  for (int j = 0; j < 4; j++){
    ultraStop += ultraStops[j];
  }

}

// =====================================================================// 
// Pin setup
// =====================================================================// 
void pinSetup(){
  // Set up motor outputs
  pinMode(enAR, OUTPUT);
  pinMode(enBL, OUTPUT);
  pinMode(in1R, OUTPUT);
  pinMode(in2R, OUTPUT);
  pinMode(in3L, OUTPUT);
  pinMode(in4L, OUTPUT);

  // Ultrasonic sensors
  pinMode(trigUltraPin, OUTPUT);
  for (int i = 0; i<4; i++){
    pinMode(echoUltraPin[i], INPUT);
  }

  // Set up speed sensor inputs
  pinMode(speedPinL, INPUT);
  pinMode(speedPinR, INPUT);
}
// =====================================================================// 
// Print time,pwmL,pwmR,speedL,speedR
// =====================================================================// 
void printSpeeds(){
  static int i = 0;
  if (!i){ // Only print at the start
    Serial.println("time,desL,pwmL,speedL,desR,pwmR,speedR");
    i = 1;
  }

  currentTime_ms = millis();
  float elapsed_time = (currentTime_ms - startTime_ms) / 1000.0;
  
  Serial.print(elapsed_time, 3); // Time (s)
  Serial.print(",");
  Serial.print(desiredSpeedL,3); // Desired Speed of left motor
  Serial.print(",");
  Serial.print(pwmL,0); // PWM Left (input)
  Serial.print(",");
  Serial.print(speedL, 3); // Left speed (output)
  Serial.print(",");
  Serial.print(desiredSpeedR,3); // Desired Speed of right motor
  Serial.print(",");
  Serial.print(pwmR,0); // PWM Right (input)
  Serial.print(",");
  Serial.println(speedR, 3); // Right speed (output)
  
}

// =====================================================================// 
// Apply PWM signals (optionally ignore the constraints, default = contrains)
// =====================================================================// 
void applyPWMs(bool constrain_pwm){
  if (constrain_pwm){
    if (desiredSpeedL != 0){pwmL = constrain(pwmL, MIN_PWM, MAX_PWM);}else{pwmL = 0;}
    if (desiredSpeedR != 0){pwmR = constrain(pwmR, MIN_PWM, MAX_PWM);}else{pwmR = 0;}
  }
  analogWrite(enAR, pwmR); // Right motor
  analogWrite(enBL, pwmL); // Left motor
}

// =====================================================================// 
// Run main communication protocol
// =====================================================================// 
void runComms(){ 
  if (Serial.available() > 0) {
      
      String msg = Serial.readStringUntil('\n');
      Serial.print("got message: ");
      Serial.print(msg);
      Serial.print(" at: ");
      Serial.println(millis());
      msg.trim(); // Remove any whitespace/newline characters
      
      // Find the comma delimiter
      int commaIndex = msg.indexOf(',');
      
      if (commaIndex > 0) {
        // Split the message into two parts
        String leftSpeedStr = msg.substring(0, commaIndex);
        String rightSpeedStr = msg.substring(commaIndex + 1);
        
        // Convert to float and assign to desired speeds
        desiredSpeedL = leftSpeedStr.toFloat();
        desiredSpeedR = rightSpeedStr.toFloat();
      }
    }
}

// =====================================================================// 
// Calculate speeds of the wheels
// =====================================================================// 
void calcSpeeds(){ 
  speedL = dtL ? (speed_coeff/dtL) : 0; // return 0 if dt = 0
  speedR = dtR ? (speed_coeff/dtR) : 0; // return 0 if dt = 0
}

// =====================================================================// 
// Interrupt functions
// =====================================================================// 
void riseL(){ // ISR call for left wheel
  static int i = 0;
  static unsigned long rise0 = 0;
  static unsigned long rise1 = 0;
  
  if (!i){ // Get start time
    i++;
    rise0 = micros();
  }else if (i == WAIT_RISES){ // Measure time delta and reset i
    rise1 = micros();
    dtL = rise1 - rise0;
    i = 0;
    lastMeasureL_us = rise1;
  }else{ // Wait for another rise
    i++;
  }
}

void riseR(){ // ISR call for right wheel
  static int i = 0;
  static unsigned long rise0 = 0;
  static unsigned long rise1 = 0;
  
  if (!i){ // Get start time
    i++;
    rise0 = micros();
  }else if (i == WAIT_RISES){// Measure time delta and reset i
    rise1 = micros();
    dtR = rise1 - rise0;
    i = 0;
    lastMeasureR_us = rise1;
  }else{ // Wait for another rise
    i++;
  }
}

// =====================================================================// 
// Change motor direction according to desired speed
// =====================================================================//
void adjustDirection(){ 
    if (desiredSpeedL < 0){reverseL();} else {forwardL();};
    if (desiredSpeedR < 0){reverseR();} else {forwardR();};
}
// =====================================================================// 
// Turn off motor
// =====================================================================//
void stopMotors(int whichMotor) { // Default = both, 0 = Left, 1 = Right
  if (whichMotor == 2 || whichMotor == 1) { // Right or Both
    analogWrite(enAR, 0);
    //digitalWrite(in1R, LOW);
    //digitalWrite(in2R, LOW);
  }
  if (whichMotor == 2 || whichMotor == 0) { // Left or Both
    analogWrite(enBL, 0);
    //digitalWrite(in3L, LOW);
    //digitalWrite(in4L, LOW);
  }
}

// =====================================================================// 
// Motor direction functions
// =====================================================================// 
void forwardR() { digitalWrite(in1R, LOW); digitalWrite(in2R, HIGH); }
void reverseR() { digitalWrite(in1R, HIGH);  digitalWrite(in2R, LOW); }
void forwardL() { digitalWrite(in4L, LOW); digitalWrite(in3L, HIGH); }
void reverseL() { digitalWrite(in4L, HIGH);  digitalWrite(in3L, LOW); }