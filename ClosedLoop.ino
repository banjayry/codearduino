
// PIN 
#define ENA   9
#define IN1   8
#define IN2   7

#define ENC_A 2      // Encoder channel A (interrupt)

#define TRIG  4
#define ECHO  5

// ENCODER 
volatile long encoderCount = 0;
long lastPulse = 0;

// Hasil kalibrasi motor
const long PPR_OUTPUT = 660;  // pulses per 360° on output shaft

// ANGLE CONTROL 
const int TARGET_ANGLE = 120; // degrees
const long TARGET_PULSE = (PPR_OUTPUT * TARGET_ANGLE) / 360;

// PID CONFIG
float Kp = 1.8;
float Ki = 4.0;
float Kd = 0.1;

float rpmSet = 30.0;        // setpoint RPM saat berputar
float rpmRaw = 0.0;
float rpmFiltered = 0.0;

float error = 0.0, prevError = 0.0;
float integral = 0.0;

float P_term = 0.0;
float I_term = 0.0;
float D_term = 0.0;

int pwmOut = 0;

// FILTER
const float ALPHA = 0.2;      // 0.1–0.3 (lebih kecil = lebih halus)

// TIMING
const unsigned long SAMPLE_TIME = 150; // ms
unsigned long lastTime = 0;

//  ULTRASONIC
const int DIST_THRESHOLD = 5; // cm

// STATE
enum State { IDLE, ROTATE };
State state = IDLE;

// ISR
void encoderISR() {
  encoderCount++;
}

// MOTOR
void motorForward(int pwm) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, pwm);
}

void motorStop() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

// ULTRASONIC
long readDistanceCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 25000); // timeout 25ms
  if (duration == 0) return 999;

  return (long)(duration * 0.034 / 2.0);
}

// SETUP
void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  motorStop();

  // IMPORTANT: Serial Plotter expects numeric streams (no headers)
  // Output order:
  // value1=setpoint, value2=rpm_actual, value3=P, value4=I, value5=D
}

// LOOP
void loop() {
  long distance = readDistanceCM();

  // ===== START CONDITION =====
  if (state == IDLE && distance < DIST_THRESHOLD) {
    noInterrupts();
    encoderCount = 0;
    interrupts();

    lastPulse = 0;
    integral = 0;
    prevError = 0;
    rpmFiltered = 0;

    P_term = I_term = D_term = 0;
    pwmOut = 0;

    state = ROTATE;
  }

  // ===== CONTROL LOOP =====
  unsigned long now = millis();
  if (now - lastTime >= SAMPLE_TIME) {
    lastTime = now;

    // ---- read encoder atomically ----
    noInterrupts();
    long pulses = encoderCount;
    interrupts();

    long deltaPulse = pulses - lastPulse;
    if (deltaPulse < 0) deltaPulse = 0; // safety
    lastPulse = pulses;

    float dt = SAMPLE_TIME / 1000.0;

    // ---- compute RPM ----
    rpmRaw = (deltaPulse / (float)PPR_OUTPUT) * (60.0 / dt);
    rpmFiltered = ALPHA * rpmRaw + (1.0 - ALPHA) * rpmFiltered;

    if (state == ROTATE) {
      // ---- PID speed ----
      error = rpmSet - rpmFiltered;

      integral += error * dt;
      integral = constrain(integral, -40, 40);

      float derivative = (error - prevError) / dt;
      prevError = error;

      P_term = Kp * error;
      I_term = Ki * integral;
      D_term = Kd * derivative;

      float control = P_term + I_term + D_term;
      pwmOut = constrain((int)control, 0, 255);

      motorForward(pwmOut);

      // angle stop (120)
      if (pulses >= TARGET_PULSE) {
        motorStop();
        state = IDLE;
        pwmOut = 0;
        // anti re-trigger
        delay(800);
      }
    } else {
      motorStop();
      pwmOut = 0;
      P_term = I_term = D_term = 0;
    }
    
    // value1 value2 value3 value4 value5 value6
    Serial.print(rpmSet);        Serial.print(' ');
    Serial.print(rpmFiltered);  Serial.print(' ');
    Serial.print(P_term);       Serial.print(' ');
    Serial.print(I_term);       Serial.print(' ');
    Serial.println(D_term);
    Serial.print(pwmOut);       Serial.print(' ');
  }
}
