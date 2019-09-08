/* DEFINES ********************************************************************/
// GRBL Pins
#define STEP  18
#define DIR   19

// Encoder Pins
#define ENCODER_CH_A  2
#define ENCODER_CH_B  3

// Motor Driver Pins
#define MOTOR_SPEED  8
#define MOTOR_DIR    9

// PID Anti-windup Term
#define I_MAX   10
#define I_MIN  -10

// PID Loop Interval
#define SAMPLING_INTERVAL 5

/* GLOBAL VARIABLES ***********************************************************/
volatile long encoderCount = 0; // number of pulses from encoder
volatile long stepsCount = 0;   // target count for the motor

/* SETUP CODE *****************************************************************/
void setup() {
  // Motor Driver Pins
  pinMode(MOTOR_SPEED, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);

  // GRBL Pins
  pinMode(STEP, INPUT);
  pinMode(DIR, INPUT);

  // Encoder Pins
  pinMode(ENCODER_CH_A, INPUT);
  pinMode(ENCODER_CH_B, INPUT);

  // turn on pull-up resistors
  digitalWrite(ENCODER_CH_A, HIGH);
  digitalWrite(ENCODER_CH_B, HIGH);

  // tun on pull-down resistors
  digitalWrite(STEP, LOW);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_CH_A), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CH_B), doEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEP), stepCount, RISING);
}

/* MAIN LOOP ******************************************************************/
void loop() {
  long targetCounts = stepsCount;
  doPID(targetCounts);
}

/* FUNCTION DEFINITION ********************************************************/
/*
 * PID Controller function
 */
void doPID(long targetCounts){
  static long previousMillis = 0; // last loop time
  static float prevError;         // last control error
  
  // Controller gains
  float Kp = 2.22;
  float Ki = 3.5;
  float Kd = 0.38;
  
  float pTerm, iTerm, dTerm;
  long error;
  long iError;
  int motorSpeed;
  
  if (millis() - previousMillis > SAMPLING_INTERVAL){
    previousMillis = millis();

    error = encoderCount - targetCounts;
    pTerm = Kp * error;
    iError += error;
    iTerm = Ki * constrain(iError, I_MIN, I_MAX);
    dTerm = Kd * (error - prevError);

    motorSpeed = constrain(pTerm + iTerm + dTerm, -255, 255);

    if(motorSpeed > 0){
      digitalWrite(MOTOR_DIR, HIGH);
    }
    if(motorSpeed < 0){
      digitalWrite(MOTOR_DIR, LOW);
      motorSpeed = -1 * motorSpeed;
    }
    analogWrite(MOTOR_SPEED, motorSpeed);
  }
}

/*
 * ISR for encoder channel A
 */
void doEncoderA(){
  if(digitalRead(ENCODER_CH_A) == HIGH){
    if(digitalRead(ENCODER_CH_B) == LOW)
      encoderCount += 1;
    else
      encoderCount -= 1;
  }
  else{
    if(digitalRead(ENCODER_CH_B) == HIGH)
      encoderCount += 1;
    else
      encoderCount -= 1;
  }
}

/*
 * ISR for encoder channel B
 */
void doEncoderB(){
  if(digitalRead(ENCODER_CH_B) == HIGH){
    if(digitalRead(ENCODER_CH_A) == HIGH)
      encoderCount += 1;
    else
      encoderCount -= 1;
  }
  else{
    if(digitalRead(ENCODER_CH_A) == LOW)
      encoderCount += 1;
    else
      encoderCount -= 1;
  }
}

/*
 * ISR to count the steps required for the motor
 */
void stepCount(){
  if(digitalRead(DIR))
    stepsCount++;
  else
    stepsCount--;
}
