#include <Adafruit_LSM6DS33.h>

// motor driver pins/parameters
#define L_MOT_STEP 3
#define L_MOT_DIR  4
#define R_MOT_STEP 5
#define R_MOT_DIR  6

// fullstep = 200SPR, halfstep = 400SPR
#define stepsPerRevolution 400
//#define stepperDelay 2000

Adafruit_LSM6DS33 lsm6ds33;


struct Sensor {
  sensors_event_t Accel;
  sensors_event_t Gyro;
  sensors_event_t Temperature;
};

void setup() {
  Serial.begin(115200);

  while(!Serial){
    delay(10);
    Serial.println("Initializing..");
  }

  if (!lsm6ds33.begin_I2C()) {
    Serial.println("Failed to find LSM6D33");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("LSM6DS33 Found");

  // set gyro LSM6DS33 parameters)
  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
  lsm6ds33.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds33.setGyroDataRate(LSM6DS_RATE_104_HZ);

  lsm6ds33.configInt1(false, false, true);
  lsm6ds33.configInt2(false, true, false);

  // motor pins declared output
  pinMode(L_MOT_STEP, OUTPUT);
  pinMode(L_MOT_DIR, OUTPUT);
  pinMode(R_MOT_STEP, OUTPUT);
  pinMode(R_MOT_DIR, OUTPUT);

  Serial.println("READY");
  Serial.println(" ");


}

long before = millis(); // make sure the loop doesnt run too fast, or consider use micros()
float accmulateError = 0;
void loop() {
  long now = millis();
  float dt = float(now - before);
  before = now;

  //update gyro and accel
  Sensor sensor = fetchSensor();

  // tuneable values
  const float kP = 1.0;
  const float kI = 0; //make it non zero once P is stable
  const float tol = 1.0;

  // calculate error and motor output
  float error = 0.0 - sensor.Gyro.gyro.z;
  // output = P + I
  accmulateError += error * dt;
  float output = (error * kP) + (accmulateError * kI);

  if (abs(output) >= tol) {
    stepperBump(output, 1000); //cw
  }

  //TODO remove delay
  delay(50);
}


// drive motor towards <dir> for <stepDelay> micros
void stepperBump(float output, int stepDelay) {
  // TODO make sure the output direction logic is right
  if(output > 0) {
    digitalWrite(L_MOT_DIR, LOW);
    digitalWrite(R_MOT_DIR, HIGH);
  } else {
    digitalWrite(L_MOT_DIR, HIGH);
    digitalWrite(R_MOT_DIR, LOW);
  }

  // assume abs output goes from 0 to 1000
  int power = map(abs(output), 0, 1000, 0, 255);
  analogWrite(L_MOT_STEP, power);
  analogWrite(R_MOT_STEP, power);
  delayMicroseconds(stepDelay);
  digitalWrite(L_MOT_STEP, 0);
  digitalWrite(R_MOT_STEP, 0);
}

// fetchSensor fetch the latest sensor reading
Sensor fetchSensor() {
  Sensor s;

  lsm6ds33.getEvent(&s.Accel, &s.Gyro, &s.Temperature);

  Serial.print("Accel ");
  Serial.print(s.Accel.acceleration.x);
  Serial.print(" \\aX ");
  Serial.print(s.Accel.acceleration.y);
  Serial.print(" \\aY" );
  Serial.print(s.Accel.acceleration.z);
  Serial.print(" \\aZ ");
  Serial.println("m/s^2");

  Serial.print("Gyro ");
  Serial.print(s.Gyro.gyro.x);
  Serial.print(" \\gX ");
  Serial.print(s.Gyro.gyro.y);
  Serial.print(" \\gY ");
  Serial.print(s.Gyro.gyro.z);
  Serial.print(" \\gZ ");
  Serial.println("rad/s");

  return s;
}
