//GRBL Pins
#define STEP 18 //ISR
#define DIR 19

// Encoder Pins
#define encoderPinA 2 //ISR
#define encoderPinB 3 //ISR

// Motor Driver Pins
#define mSpeed 8
#define mDir 9

volatile long encoderCount = 0;
volatile long stepsCount = 0;
long targetCounts = 0;

// PID
float Kp, Ki, Kd;
float prevError, iError;
float pTerm, iTerm, dTerm;
long error;
long previousMillis = 0;
long interval = 5;
int motorSpeed;
int iMin = -10;
int iMax = 10;


void setup() {
  // Motor Driver Pins
  pinMode(mSpeed, OUTPUT);
  pinMode(mDir, OUTPUT);

  // GRBL Pins
  pinMode(STEP, INPUT);
  pinMode(DIR, INPUT);

  // Encoder Pins
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // turn on pull-up resistors
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);

  // tun on pull-down resistors
  digitalWrite(STEP, LOW);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEP), stepCount, RISING);

  // Controller Gains
  Kp = 2.22;
  Ki = 3.5;
  Kd = 0.38;
}

void loop() {
  targetCounts = stepsCount;
  doPID();
}

void doPID(){
  if (millis() - previousMillis > interval){
    previousMillis = millis();

    error = encoderCount - targetCounts;
    pTerm = Kp * error;
    iError += error;
    iTerm = Ki * constrain(iError, iMin, iMax);
    dTerm = Kd * (error - prevError);

    motorSpeed = constrain(pTerm + iTerm + dTerm, -255, 255);

    if(motorSpeed > 0){
      digitalWrite(mDir, HIGH);
    }
    if(motorSpeed < 0){
      digitalWrite(mDir, LOW);
      motorSpeed = -1 * motorSpeed;
    }
    analogWrite(mSpeed, motorSpeed);
  }
}

// Interrput Service Routines
void doEncoderA(){
  if(digitalRead(encoderPinA) == HIGH){
    if(digitalRead(encoderPinB) == LOW)
      encoderCount += 1;
    else
      encoderCount -= 1;
  }
  else{
    if(digitalRead(encoderPinB) == HIGH)
      encoderCount += 1;
    else
      encoderCount -= 1;
  }
}

void doEncoderB(){
  if(digitalRead(encoderPinB) == HIGH){
    if(digitalRead(encoderPinA) == HIGH)
      encoderCount += 1;
    else
      encoderCount -= 1;
  }
  else{
    if(digitalRead(encoderPinA) == LOW)
      encoderCount += 1;
    else
      encoderCount -= 1;
  }
}

void stepCount(){
  if(digitalRead(DIR))
    stepsCount++;
  else
    stepsCount--;
}
