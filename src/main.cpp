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
void runTesterMode();
int sign(float x);
float scaling_tank(float x);

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
#define MIN_PWM 0 
#define MAX_PWM 255 // Max allowed pwm
#define WAIT_RISES 1 // # rising edges until new speed can be measured (excluding first) 

// =====================================================================//
// Global speed measurements values
float speedL = 0;
float speedR = 0; // m/s

// =====================================================================//
// Global speed requirements
volatile float desiredSpeedL = 0; // m/s
volatile float desiredSpeedR = 0; // m/s
float prevDesL = 0;
float prevDesR = 0;
volatile int directL = 1 ; // forward
volatile int directR = 1 ; // forward

// =====================================================================//
// System control variables

// PWM inputs
float pwmL = 70; // Start at higher value to prevent windup
float pwmR = 70; // Start at higher value to prevent windup

// PID parameters
float KpL = 70, KiL = 0, KdL = 17;
float KpR = 70, KiR = 0, KdR = 17;

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
unsigned long measureTimeout_us = 500000; // If no new interrupts, reset speed to 0 after this time

// =====================================================================//
// Speed measurement coeeficient
const float radius = 0.0325; // wheel radius [m]
const float speed_coeff = (radius)*(2*M_PI/GAPS*WAIT_RISES)*(1e6);
                        //(radius)*(dtheta)*(u_sec/s) 

// =====================================================================//
// Timing variables
unsigned long currentTime_ms = millis();
unsigned long startTime_ms ;
unsigned long currentTime_us = micros();
// =====================================================================//
// Tester variables
bool closedLoop = true; // Should the system run the closedloop system?
bool echoSpeeds = false; // Should the system echo the speeds to terminal (For tuning)
bool ultraEnabled = false; // Should we run ultrasonic measurements?
bool ultraStopEnabled = false; // Should the car stop if dist < threshold?
bool commsEnabled = true; // Should the arduino send data to the Jetson?
bool tester_mode = false; // Should the system run test mode (set speed to certain amount for a time)
  unsigned long tester_start = 0 ;
  float deg = 45;
  float scaled = scaling_tank(deg);
  unsigned long turntime = 0.6*M_PI/180 * (scaled) * 1000;
  unsigned long timings[] = {6000}; // Run corresponding speed UP TO this time
  float speeds[][2] = {{0.2, 0.2}};
// =====================================================================//
// Ultrasonic measurements
float ultraDist[4] = {100,100,100,100}; // Distances in meters
float stopThres[4] = {20,10,10,10}; // cm
float ultraStops[4] = {0, 0 ,0 ,0}; // Flag which will stop the motor if too close
float ultraStop = 0;
// =====================================================================//
// Final adjustments
int start_final = 0;
int clockwise;
float tanktime;
float straighttime;
float time_passed_final = 0;
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

  if (echoSpeeds){
    runMeasureComms();
  }

  prevDesL = desiredSpeedL;
  prevDesR = desiredSpeedR;

  applyPWMs(); // Apply specified pwm signals with constraints
  
}

// =====================================================================// 
// Main loop
// =====================================================================// 
void loop(){
  static int dt_ms = 0;
  static float dt_s = 0;
  static unsigned long prevTime_ms = 0;
  currentTime_ms = millis();
  static unsigned long ultraPrev_us = 0;

  if (tester_start == 0){
    tester_start = millis();
  }

  if (tester_mode){
    runTesterMode();
  }

  if (start_final){
    if (desiredSpeedL != 0.1 || desiredSpeedR != 0.1){
      desiredSpeedL = 0.1;
      desiredSpeedR = 0.1;
    }
    time_passed_final = (millis() - start_final)/1000.0;
    if (time_passed_final < tanktime){
      // Tank turn phase
      if (clockwise){directL = 1; directR = 0;}else{directL = 0; directR = 1;}
    }
    else if (time_passed_final < tanktime+1){
      // 1-second pause after tank turn
      desiredSpeedL = 0;
      desiredSpeedR = 0;
    }
    else if (time_passed_final < tanktime+1+straighttime){
      // Straight movement phase  
      desiredSpeedL = 0.1;
      desiredSpeedR = 0.1;
      directL = 1;
      directR = 1;
    }
    else{
      // End sequence
      desiredSpeedL = 0;
      desiredSpeedR = 0;
      start_final = 0;
    }
  }

  adjustDirection(); // Sets the motors in correct direction

  if (commsEnabled){ // Run communication to Jetson
    runComms();
  }
  
  if ((prevDesL == 0) && (desiredSpeedL != 0)){
    pwmL = 30;
  }
  if ((prevDesR == 0) && (desiredSpeedR != 0)){
    pwmR = 30;
  }
  
  currentTime_us = micros();
  if ((ultraEnabled)&&(currentTime_us - ultraPrev_us > 10000)){ // Run ultrasonic sensors only in 10ms intervals
    ultraPrev_us = currentTime_us;
    runUltras();
  }

  if (!ultraStop){
    if (closedLoop){
      
      dt_ms = currentTime_ms - prevTime_ms ; // Update elapsed time
      if (dt_ms > TS){ // if elapsed time > sampling time

        prevTime_ms = currentTime_ms;
        dt_s = dt_ms / 1000.0; 

        calcSpeeds(); // Update speeds

        currentTime_us = micros();

        if (currentTime_us - lastMeasureL_us > measureTimeout_us){
          //dtL = 0; // speed calc handles 0 dt as 0 speed
          speedL = 0;
        }
        if (currentTime_us - lastMeasureR_us > measureTimeout_us){
          //dtR = 0;
          speedR = 0;
        }

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

  prevDesL = desiredSpeedL;
  prevDesR = desiredSpeedR;
}

// =====================================================================// 
// Tester mode function
// =====================================================================// 
void runTesterMode(){
  static unsigned long time_passed;
  time_passed = millis() - tester_start;
  
  for (unsigned int i = 0; i<sizeof(timings)/sizeof(timings[0]); i++) {
    if (time_passed < timings[i]) {
      desiredSpeedL = abs(speeds[i][0]);
      desiredSpeedR = abs(speeds[i][1]);
      directL = sign(speeds[i][0]);
      directR = sign(speeds[i][1]);
      break;
    }
  }
  if (time_passed > timings[sizeof(timings)/sizeof(timings[0])-1]){
    desiredSpeedL = 0;
    desiredSpeedR = 0;
    directL = 1;
    directR = 1;
  }

  // Set target speeds based on time
  /*if (time_passed < 1000){ // forwards
    desiredSpeedL = 0.2;
    desiredSpeedR = 0.2;
    directL = 1;
    directR = 1;
  }
  else if (time_passed < 0000){ // first command
    desiredSpeedL = 0.2;
    desiredSpeedR = 0.1;
    directL = 1;
    directR = 1;
  }
  else if (time_passed < 0000){ // second command
    desiredSpeedL = 0.1;
    desiredSpeedR = 0.2;
    directL = 1;
    directR = 1;
  }
  else if (time_passed < 0000){ // third command
    desiredSpeedL = 0.2;
    desiredSpeedR = 0.2;
    directL = 1;
    directR = 1;
  }
  else{ // stop
    desiredSpeedL = 0;
    desiredSpeedR = 0;
    directL = 1;
    directR = 1;
  }*/
}

// =====================================================================// 
// Sign function
// =====================================================================//
int sign(float x){
  if (x < 0) return 0;
  if (x >= 0) return 1;
  return 1;
}

// =====================================================================// 
// Tank scaling function
// =====================================================================//
float scaling_tank(float x){
  
  float polynomial = -4.32307692308e-7 * pow(x, 5) + 
                     6.22319347319e-5 * pow(x, 4) - 
                     0.00346042540793 * pow(x, 3) + 
                     0.0931500291375 * pow(x, 2) - 
                     1.2352733683 * x + 
                     7.73791666667;
  
  return polynomial * x;
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
/*void runUltras(){
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

}*/
void runUltras(){
  static int duration = 0;
  static int i = 0;

  if (i!=3){i++;}else{i = 0;}

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
  float elapsed_time = (currentTime_ms - tester_start) / 1000.0;
  float spdR = directR ? speedR : -speedR;
  float spdL = directL ? speedL : -speedL;
  float desL = directL ? desiredSpeedL : -desiredSpeedL;
  float desR = directR ? desiredSpeedR : -desiredSpeedR;
  
  Serial.print(elapsed_time, 3); // Time (s)
  Serial.print(",");
  Serial.print(desL,3); // Desired Speed of left motor
  Serial.print(",");
  Serial.print(pwmL,0); // PWM Left (input)
  Serial.print(",");
  Serial.print(spdL, 3); // Left speed (output)
  Serial.print(",");
  Serial.print(desR,3); // Desired Speed of right motor
  Serial.print(",");
  Serial.print(pwmR,0); // PWM Right (input)
  Serial.print(",");
  Serial.println(spdR, 3); // Right speed (output)
  
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
      msg.trim(); // Remove any whitespace/newline characters
      
      // Find the comma delimiters
      int firstComma = msg.indexOf(',');
      int secondComma = msg.indexOf(',', firstComma + 1);
      
      if (firstComma > 0 && secondComma > firstComma) {
        // Split the message into three parts
        String num1Str = msg.substring(0, firstComma);
        String num2Str = msg.substring(firstComma + 1, secondComma);
        String flagStr = msg.substring(secondComma + 1);
        
        // Convert to values
        float num1 = num1Str.toFloat();
        float num2 = num2Str.toFloat();
        int flag = flagStr.toInt();
        
        // Assign to desired speeds if flag == 1, else tank turn sequence
        if (flag){
          desiredSpeedL = num1;
          if (desiredSpeedL < 0){directL = 0;} else {directL = 1;};
          desiredSpeedR = num2;
          if (desiredSpeedR < 0){directR = 0;} else {directR = 1;};

          Serial.print("Received Speeds: ");
          Serial.print(desiredSpeedL);
          Serial.print(", ");
          Serial.print(desiredSpeedR);
          Serial.print(" at: ");
          Serial.print(millis()/1000.0);
          Serial.print(" Current Speed: ");
          Serial.print(speedL);
          Serial.print("|");
          Serial.print(speedR);
          Serial.print(" PWMs: ");
          Serial.print(pwmL);
          Serial.print(", ");
          Serial.print(pwmR);
        }else{ // otherwise run final adjustments
          start_final = millis();
          tanktime = abs(num1);
          if (num1 > 0){clockwise = 1;}else{clockwise = 0;}
          straighttime = num2;
        }
        
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
    if (directL == 0){reverseL();} else {forwardL();};
    if (directR == 0){reverseR();} else {forwardR();};
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