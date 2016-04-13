#include <Servo.h>
#include <Wire.h> //I2C Arduino Library


// ================================================================
// ===                   IMU MPU6050 START                      ===
// ================================================================

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

MPU6050 mpu(0x69);
//IMU addresses
#define address 0x1E //0011110b, I2C 7bit address of HMC5883 

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ===     INTERRUPT DETECTION ROUTINE    ===
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
void read_state(float state[]);
void apply_action(float action[]);
void read_output(float output[]);

// ================================================================
// ===                     IMU MPU6050 END                      ===
// ================================================================


 // IMU pins define
 int IMU_1 = 14;
 int IMU_2 = 15;
 int IMU_3 = 16;
 int IMU_4 = 17;
  
//servo motors
Servo motor_hip_left;
Servo motor_hip_right;
Servo motor_knee_left;
Servo motor_knee_right;

// Initial Angles
float init_thetaBhip = -10*3.1416/180;
float init_thetaBknee = 0;
float init_thetaFhip = 10*3.1416/180;
float init_thetaFknee = 0;

float val[4]    = {0.0,0.0,0.0,0.0};
float state[4]  = {0.0,0.0,0.0,0.0};
float action[4] = {0.0,0.0,0.0,0.0};
float output[2] = {0.,0.};
float reward    = 0;
float t = 0.001;

// ================================================================
// ===                        VOID SETUP                        ===
// ================================================================

void setup() {

// ================================================================
// ===                        IMU START                         ===
// ================================================================

  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
//        TWBR = 24; // 400kHz I2C clock (200kHz if CPU /is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

//    // wait for ready
//    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//    while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());                 // wait for data
//    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

//  Serial.begin(9600);
  motor_hip_left.attach(3);
  motor_hip_right.attach(5); // Offset of 98 for pin 5
  motor_knee_left.attach(9);
  motor_knee_right.attach(6);

  // pins for selecting the IMUs
  pinMode(IMU_1,OUTPUT);
  pinMode(IMU_2,OUTPUT);
  pinMode(IMU_3,OUTPUT);
  pinMode(IMU_4,OUTPUT);
  
  //Initialize Serial and I2C communications
//  Wire.begin();
//  //Put the HMC5883 IC into the correct operating mode
//  Wire.beginTransmission(address); //open communication with HMC5883
//  Wire.write(0x02); //select mode register
//  Wire.write(0x00); //continuous measurement mode
//  Wire.endTransmission();

}  


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  
  read_state(state); //Read state
  Serial.print("state: ");
  Serial.println(state[1]);

//  action = calculate_action(state); // apply policy
  apply_action(action); // feed the calculated action to motors
  delay(1000);
  read_output(output); // read bot speed, direction etc.
  reward = calculate_reward(output);
//  update_policy(state, action, output);

}
