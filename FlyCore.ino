#include <PID_v1.h>
#include <MPU6050.h>
#include <RF24.h>
#include <Servo.h>

Servo m1;
Servo m2;
Servo m3;
Servo m4;

double pitchInput;
double pitchOutput;
double pitchSetpoint;

double pitchKp(1);  // TUNE THIS IF NEEDED.
double pitchKi(0.5);  // TUNE THIS IF NEEDED.
double pitchKd(0.2);  // TUNE THIS IF NEEDED.

PID pitchPid(&pitchInput, &pitchOutput, &pitchSetpoint, pitchKp, pitchKi, pitchKd, DIRECT);

double rollInput;
double rollOutput;
double rollSetpoint;

double rollKp(1);  // TUNE THIS IF NEEDED.
double rollKi(0.5);  // TUNE THIS IF NEEDED.
double rollKd(0.2);  // TUNE THIS IF NEEDED.

PID rollPid(&rollInput, &rollOutput, &rollSetpoint, rollKp, rollKi, rollKd, DIRECT);

double yawInput;
double yawOutput;
double yawSetpoint;

double yawKp(1);  // TUNE THIS IF NEEDED.
double yawKi(0.5);  // TUNE THIS IF NEEDED.
double yawKd(0.2);  // TUNE THIS IF NEEDED.

PID yawPid(&yawInput, &yawOutput, &yawSetpoint, yawKp, yawKi, yawKd, DIRECT);

double throttleInput(0);
double throttleOutput;
double throttleSetpoint;

bool arm(0);

double throttleKp(1);  // TUNE THIS IF NEEDED.
double throttleKi(0.5);  // TUNE THIS IF NEEDED.
double throttleKd(0.2);  // TUNE THIS IF NEEDED.

PID throttlePid(&throttleInput, &throttleOutput, &throttleSetpoint, throttleKp, throttleKi, throttleKd, DIRECT);

MPU6050 mpu;

RF24 radio(8, 7);

const uint64_t address(0xFFFFFFFFFFFFFFFF); // CHANGE THIS TO YOUR DESIRED CODE IF NEEDED.

unsigned long prevTime;

float gRoll(0);
float gPitch(0);

const float alpha(0.98);

const float tau(2);  // TUNE THIS IF NEEDED.

void setup() {
  // SETUP CODE:
  pitchPid.SetOutputLimits(-63, 63);
  rollPid.SetOutputLimits(-63, 63);
  yawPid.SetOutputLimits(-63, 63);
  throttlePid.SetOutputLimits(0, 63);

  pitchPid.SetMode(AUTOMATIC);
  rollPid.SetMode(AUTOMATIC);
  yawPid.SetMode(AUTOMATIC);
  throttlePid.SetMode(AUTOMATIC);

  pitchPid.SetSampleTime(5);
  rollPid.SetSampleTime(5);
  yawPid.SetSampleTime(5);
  throttlePid.SetSampleTime(5);

  mpu.initialize();

  int16_t ax, ay, az, gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float newAx((ax / 16384.0) * 9.80665);
  float newAy((ay / 16384.0) * 9.80665);
  float newAz((az / 16384.0) * 9.80665);

  float aRoll  = atan2(newAy, newAz) * (180.0 / PI);
  float aPitch = atan2(-newAx, sqrt(newAy * newAy + newAz * newAz)) * (180.0 / PI);

  gRoll = aRoll;
  gPitch = aPitch;

  radio.begin();
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(110);  // CHANGE THIS TO YOUR DESIRED CHANNEL IF NEEDED.
  radio.openReadingPipe(0, address);
  radio.startListening();

  m1.attach(10);
  m2.attach(9);
  m3.attach(6);
  m4.attach(5);

  prevTime = micros();
}

void loop() {
  // MAIN CODE:
  unsigned long now = micros();
  float dt = (now - prevTime) / 1000000.0;
  prevTime = now;

  int16_t ax, ay, az, gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float newAx((ax / 16384.0) * 9.80665);
  float newAy((ay / 16384.0) * 9.80665);
  float newAz((az / 16384.0) * 9.80665);

  float newGx(gx / 131.0);
  float newGy(gy / 131.0);

  yawInput = gz / 131.0;

  float aRoll(atan2(newAy, newAz) * (180 / PI));
  float aPitch = atan2(-newAx, sqrt(newAy * newAy + newAz * newAz)) * (180.0 / PI);

  gRoll += newGx * dt;
  gPitch += newGy * dt;

  rollInput = alpha * gRoll + (1 - alpha) * aRoll;
  pitchInput = alpha * gPitch + (1 - alpha) * aPitch;

  float rollRad(rollInput * (PI / 180.0));
  float pitchRad(pitchInput * (PI / 180.0));

  float newVerticalAccel(-newAx * sin(pitchRad) + newAy * sin(rollRad) * cos(pitchRad) + newAz * cos(rollRad) * cos(pitchRad));

  newVerticalAccel -= 9.80665;

  if (abs(newVerticalAccel) < 0.05) {
    newVerticalAccel = 0;
  }

  float bleed(exp(-dt / tau));

  throttleInput = bleed * throttleInput + newVerticalAccel * dt;

  if (radio.available()) {
    int8_t reading[5];

    radio.read(&reading, sizeof(reading));

    pitchSetpoint = reading[0];
    rollSetpoint = reading[1];
    yawSetpoint = reading[2];
    throttleSetpoint = map(reading[3], -100, 100, -200, 200) / 100.0;

    arm = reading[4];

  } else {
    pitchSetpoint = 0;
    rollSetpoint = 0;
    yawSetpoint = 0;
    throttleSetpoint = 0;
    arm = 0;
  }

  if (arm) {
    pitchPid.Compute();
    rollPid.Compute();
    yawPid.Compute();
    throttlePid.Compute();

    int roundedPitchOutput(round(pitchOutput));
    int roundedRollOutput(round(rollOutput));
    int roundedYawOutput(round(yawOutput));
    int roundedThrottleOutput(round(throttleOutput));

    int m1Val(0);
    int m2Val(0);
    int m3Val(0);
    int m4Val(0);

    if (roundedPitchOutput > 0) {
      m3Val += roundedPitchOutput;
      m4Val += roundedPitchOutput;
    } else if (roundedPitchOutput < 0) {
      m1Val += -roundedPitchOutput;
      m2Val += -roundedPitchOutput;
    }

    if (roundedRollOutput > 0) {
      m1Val += roundedRollOutput;
      m3Val += roundedRollOutput;
    } else if (roundedRollOutput < 0) {
      m2Val += -roundedRollOutput;
      m4Val += -roundedRollOutput;
    }

    if (roundedYawOutput > 0) {
      m2Val += roundedYawOutput;
      m3Val += roundedYawOutput;
      m1Val += -roundedYawOutput;
      m4Val += -roundedYawOutput;
    } else if (roundedYawOutput < 0) {
      m1Val += -roundedYawOutput;
      m4Val += -roundedYawOutput;
      m2Val += roundedYawOutput;
      m3Val += roundedYawOutput;
   }

    m1Val += roundedThrottleOutput;
    m2Val += roundedThrottleOutput;
    m3Val += roundedThrottleOutput;
    m4Val += roundedThrottleOutput;

    m1Val = constrain(m1Val, 0, 255);
    m2Val = constrain(m2Val, 0, 255);
    m3Val = constrain(m3Val, 0, 255);
    m4Val = constrain(m4Val, 0, 255);

    m1.writeMicroseconds(map(m1Val, 0, 255, 1000, 2000));
    m2.writeMicroseconds(map(m2Val, 0, 255, 1000, 2000));
    m3.writeMicroseconds(map(m3Val, 0, 255, 1000, 2000));
    m4.writeMicroseconds(map(m4Val, 0, 255, 1000, 2000));
  } else {
    m1.writeMicroseconds(1000);
    m2.writeMicroseconds(1000);
    m3.writeMicroseconds(1000);
    m4.writeMicroseconds(1000);
  }
}