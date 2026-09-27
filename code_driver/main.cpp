/**
 * ESP32 position motion control example with magnetic sensor
 */
#include <SimpleFOC.h>
#include <TemperatureSensor.h>

// SPI Magnetic sensor instance (AS5047U example)

int CS_ = 7;
int MISO_  = 5;
int MOSI_ = 6;
int SCK_ = 15;

#define RS485_DIR_PIN 1

// magnetic sensor instance - SPI

MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, CS_, MISO_, MOSI_, SCK_);


float beta = 3434.0; float t0_c = 25.0;  float r0 = 10000.0; float r_fixed = 10000.0; float v_in = 3.3;
int pin_ntc = 4;

TemperatureSensor temp_sensor = TemperatureSensor(beta, t0_c, r0, r_fixed,  v_in);

#define INH_A 14 //gpio D10
#define INH_B 12 //gpio D9
#define INH_C 10 //gpio D8
#define INL_A 13  //gpio A3
#define INL_B 11 //gpio A6
#define INL_C 9 //gpio A5

#define EN_GATE 21 // GPIO 21
#define M_PWM 35 //

// Motor instance
BLDCMotor motor = BLDCMotor(21, 3.0);
BLDCDriver6PWM driver = BLDCDriver6PWM(INH_A, INL_A, INH_B, INL_B, INH_C, INL_C, EN_GATE);

#define SO1_fan 2
#define SO2_fan 42

// angle set point variable
//float target_angle = 0;
// instantiate the commander
Commander command = Commander(Serial);
//void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void setup() {
  
  // use monitoring with serial 
  Serial.begin(115200); // For USB C

  //Serial1.begin(115200, SERIAL_8N1, 44, 43); // RX=44, TX=43 for RS485 
  _delay(2000);
  SimpleFOCDebug::enable(&Serial);
  pinMode(RS485_DIR_PIN, OUTPUT);
  digitalWrite(RS485_DIR_PIN, LOW);  // Receive mode initially
  // enable more verbose output for debugging
  // comment out if not needed
  //SimpleFOCDebug::enable(&Serial);

  // initialise magnetic sensor hardware
  sensor.init();
  analogReadResolution(12);
  pinMode(M_PWM, OUTPUT);
  digitalWrite(M_PWM, LOW);

  pinMode(SO1_fan, OUTPUT);
  pinMode(SO2_fan, OUTPUT);
  digitalWrite(SO1_fan, LOW);
  digitalWrite(SO2_fan, LOW);
  
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 20;
  driver.pwm_frequency = 20000;

  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 5.0;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //motor.foc_modulation = FOCModulationType::SinePWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity_openloop;

  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

// Limits 
  motor.velocity_limit = 1.0; 
  motor.current_limit = 40;   

  // initialize motor
  motor.init();
  // align encoder and start FOC
  //motor.zero_electric_angle  = 1.0071; // rad
  //motor.sensor_direction = Direction::CW; // CW or CCW

  motor.initFOC();


  Serial.println("Sensor zero offset is:");
  Serial.println(motor.zero_electric_angle, 4);
  Serial.println("Sensor natural direction is: ");
  Serial.println(motor.sensor_direction == Direction::CW ? "Direction::CW" : "Direction::CCW");

  motor.target = 0.0; //initial target velocity 1 rad/s
  
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
}


bool readSerial1(char* msg, int& msg_cnt){
  
  try{
    while (Serial.available()) 
    {
      // get the new byte:
      char ch = Serial.read();
      // end of user input
      if (ch == '\n') {
        return 1;
      }
      msg[msg_cnt] = ch;
      msg_cnt += 1;
      if (msg_cnt >20){
        msg_cnt = 0;
        return 1;
      }
    }
  }
  catch (...){
  }


  return 0;
}


int i_ = 0;

const int max_input_lenght = 20;
char received_chars[max_input_lenght+3] = {0}; //!< so far received user message - waiting for newline
int rec_cnt = 0; //!< number of characters receives
//String msg = "";
int msg_length = 0;

int int_tansmit = 1;

unsigned long previousMicros = micros();    // Stores the last time loop ran
unsigned long currentMicros = micros();    // Stores the last time loop ran

float angle_s = 0.0;

float temp_s = 0.0;
float result__ = 0.0;
int why;

void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  //motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  //motor.move(target_angle);
  motor.loopFOC();
  
  
  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();


  if (int_tansmit == 0){
    
    if (readSerial1(received_chars, rec_cnt)){
      int_tansmit = 1;
      if (received_chars[0] == 'T'){
        // Create a buffer to hold the substring
        char substring[rec_cnt] = {0}; // +1 for the null terminator
        //memset(substring, 0, rec_cnt); // Ensure it's null-terminated
        // Copy the characters from received_chars to substring
        for (int i = 0; i < rec_cnt-1; i++) {
          substring[i] = received_chars[1 + i];
        }
        try{
        float result = atof(substring); 
        result__ = result;
        motor.target = result;
        
        }catch(...){
          why = 1;
        }
      } 
      
      
     rec_cnt = 0;
    }
  }
  if (rec_cnt >= max_input_lenght){
    rec_cnt = 0;
  }

  
  if (int_tansmit == 1) {
    angle_s = sensor.getAngle();
    int adcValue = analogRead(pin_ntc);
    float v_termistor = adcValue * (3.33 / 4095.0);
    temp_s = temp_sensor.readTemp(v_termistor); 
    digitalWrite(RS485_DIR_PIN, HIGH); // Transmit mode
    int_tansmit = 2;
    previousMicros = micros();
  }

  currentMicros = micros();
  if (int_tansmit == 2){
    if (currentMicros - previousMicros >= 100){
      Serial.print(temp_s);
      Serial.print(",");
      Serial.println(angle_s);
      Serial.flush();
      int_tansmit = 3;
      digitalWrite(RS485_DIR_PIN, LOW);  // Back to receive mode
      previousMicros = micros();
    }
  }

  currentMicros = micros();
  if (int_tansmit == 3){
    if (currentMicros - previousMicros >= 10){
      int_tansmit = 0;
      digitalWrite(RS485_DIR_PIN, LOW);  // Back to receive mode
    }
  }

  


  motor.move();

  // user communication
  //command.run();
}