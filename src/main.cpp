#include <SimpleFOC.h>

#include "balancer_pinouts.h"
// #include "imu_helpers.h"
#include "ESP_NOW_helper.h"
 





// create motor and driver instances
BLDCMotor motor1 = BLDCMotor(11); // left
BLDCMotor motor2 = BLDCMotor(11); // right
BLDCDriver3PWM driver1 = BLDCDriver3PWM(MOT1_A, MOT1_B, MOT1_C, MOT1_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(MOT2_A, MOT2_B, MOT2_C, MOT2_EN);

MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5147_SPI, 21);
MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5147_SPI, 14); // zero for the left leg and 1 for the right leg

// imu spi pin 4
int intFlag = 0;
int initFOC_ready = 1; 
float pitch = 0;
float vel = 0;
int dir = 0;
int timeout = 0;

struct_velocity data_vel;
struct_mot_data data_out;

// hw_timer_t *My_timer = NULL;
// void IRAM_ATTR onTimer() {
//   intFlag = 1;
  // mpu6050.update();
  // float angle = 5;//mpu6050.getAngleX();  // mpu.readAngleDegree();
  // Serial.printf("Angle, %.2f\r\n", angle);
  // NowSerial.printf("Angle, %.2f\r\n", angle);  // print some text to the serial consol.
  // NowSerial.println(angle);    // read the angle value from the AS5047P sensor an print it to the serial consol.
// }

void IRAM_ATTR isr() {
  Serial.println("Reset detected");
  ESP.restart();
}


// control algorithm parameters
// stabilisation pid
PIDController pid_stb = PIDController(50.0, 100, 1, 100000, 4); 
// pid_stb.P = 30;
// pid_stb.I = 100;
// pid_stb.D = 1;
// pid_stb.ramp = 100000;
// pid_stb.limit = 7;
// velocity pid
PIDController pid_vel = PIDController(0.005, 0.01, 0, 1000, _PI*0.1);
// velocity control filtering
LowPassFilter lpf_pitch_cmd = LowPassFilter(0.5);
// low pass filters for user commands - throttle and steering
LowPassFilter lpf_throttle = LowPassFilter(0.5);
LowPassFilter lpf_steering = LowPassFilter(0.1);

// Bluetooth app variables
float steering = 0;
float throttle = 0;
float max_throttle = 20; // 20 rad/s
float max_steering = 1; // 1 V
int state = 1; // 1 on / 0 off

// motion control tunning using commander
Commander commander = Commander(Serial);
void cntStab(char* cmd) {  commander.pid(&pid_stb, cmd);}
void cntMove(char* cmd) {  commander.pid(&pid_vel, cmd);}
void lpfPitch(char* cmd) {  commander.lpf(&lpf_pitch_cmd, cmd);}
void lpfSteering(char* cmd) {  commander.lpf(&lpf_steering, cmd);}
void lpfThrottle(char* cmd) {  commander.lpf(&lpf_throttle, cmd);} 


void handleBluetooth(Stream& bt_port);
int itter = 0;

void setup() {
  // use monitoring with Serial for motor init
  // monitoring port
  Serial.begin(250000);
  
  attachInterrupt(9, isr, FALLING);
  digitalWrite(21, HIGH);
  digitalWrite(14, HIGH);
  // SimpleFOCDebug::enable(&Serial);

  _delay(1000);
  WiFi.mode(WIFI_STA);
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());
  // imu init and configure
//   if ( !initIMU() ) {
//     Serial.println(F("IMU connection problem... Disabling!"));
//  //   NowSerial.println(F("IMU connection problem... Disabling!"));
//     return;
//   }
//   _delay(1000);

  // initialise encoder hardware
  if(WiFi.macAddress() == "E4:B0:63:43:38:88") { 
    Serial.println("Right leg");
    dir = 0;
  } else if(WiFi.macAddress() == "E4:B0:63:43:40:88") {
    Serial.println("Left leg");
    dir = 1;
  } else {
    Serial.println("Unknown leg");

  }

  sensor1.init();
  Serial.println(dir);
  // encoder1.enableInterrupts(doA1, doB1);
  sensor2.init(dir);
  // encoder2.enableInterrupts(doA2, doB2);


  // link the motor to the sensor
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  // power supply voltage [V]
  driver1.pwm_frequency = 20000;
  driver1.voltage_power_supply = 12;
  if(!driver1.init()) {
    Serial.println("Driver 1 init failed");
    while(1);
  }
  motor1.linkDriver(&driver1);
  driver2.pwm_frequency = 20000;
  driver2.voltage_power_supply = 12;
  if(!driver2.init()) {
    Serial.println("Driver 2 init failed");
    while(1);
  }
  motor2.linkDriver(&driver2);

  // set control loop type to be used
  // using voltage torque mode 
  motor1.voltage_sensor_align = 2; 
  motor2.voltage_sensor_align = 2;
  motor1.LPF_velocity.Tf = 0.01; // 10ms low pass filter time constant, default 5ms
  motor2.LPF_velocity.Tf = 0.075; // 75ms low pass filter time constant, default 5ms
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.controller = MotionControlType::torque;
  motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor2.controller = MotionControlType::torque;


  // enable monitoring
  // motor1.useMonitoring(Serial);
  // motor2.useMonitoring(Serial);

  // initialise motor

  motor1.init();
  motor2.init();
  
  // MOT: Zero elec. angle: 4.96
  // motor1.sensor_direction = CW;
  // motor1.zero_electric_angle = 2.23;
  // motor2.sensor_direction = CW;
  // motor2.zero_electric_angle = 4.60;
    // align encoder and start FOC
  Serial.println("Init motor 1 (left)");
  initFOC_ready = motor1.initFOC(); 
  Serial.println("Init motor 2 (right)");
  initFOC_ready = motor2.initFOC();
  
  if(!initFOC_ready){
    Serial.println("Init FOC failed");
    // while(1);
  }
  // motor1.target = 2;
  // motor2.target = 2;

  // Serial.println(motor1.sensor_direction);
  delay(1000);

  // add the configuration commands
  commander.add('A', cntStab, "pid stab");
  commander.add('B', cntMove, "pid vel");
  commander.add('C', lpfThrottle, "lpf vel command");
  commander.add('D', lpfPitch, "lpf throttle");
  commander.add('E', lpfSteering, "lpf steering");

  Serial.println(F("Balancing robot ready!"));
  // NowSerial.println(F("Balancing robot ready!"));
  espNowInit();
  timeout = millis();
}


void loop() {
  // iterative setting FOC phase voltage
  // motor1.loopFOC();
  motor2.loopFOC();
  
  

  // iterative function setting the outter loop target
  // motor1.move();
  motor2.move();
  // Serial.printf("State: %i\n\r", state);
  if (!state || (millis() - timeout > 1000)) { // if balancer disabled
    // motor1.target = 0;
    motor2.target = 0;
  // } else if ( hasDataIMU() ) { // when IMU has received the package
  }  if (msgData()) {
      // msgFlag = 0;
    // read pitch from the IMU
    // float pitch = getPitchIMU();

    data_vel = getMsg();
    pitch = data_vel.vel_L;
    vel = data_vel.vel_R;
    timeout = millis();
    // pitch = getPitchMsg(); 
  } else {
    return;
  }

    // Serial.printf("\tPitch: %.2f\t", pitch);
    // calculate the target angle for throttle control
    // float target_pitch = lpf_pitch_cmd(pid_vel((motor1.shaft_velocity + motor2.shaft_velocity) / 2 - lpf_throttle(throttle)));
    // float target_pitch = lpf_pitch_cmd(pid_vel(-motor2.shaft_velocity - lpf_throttle(throttle)));
    float target_pitch = lpf_pitch_cmd(pid_vel(vel/2 - lpf_throttle(throttle)));
    // calculate the target voltage
    float voltage_control = pid_stb(target_pitch+pitch);//
    
    // filter steering
    float steering_adj = lpf_steering(steering);
    // set the tergat voltage value 
    // motor1.target = voltage_control; // + steering_adj;
    motor2.target = voltage_control; // - steering_adj;
 
    // if( itter++ % 100 == 0){

      // Serial.printf("as2: %.2f,a2: %.2f, shaft_vel: %.2f, sensor_vel: %.2f \t\tPitch: %.2f\tTarget Voltage: %.2f\tTarget pitch: %.2f\n\r"
      //   ,motor2.shaft_angle, sensor2.getAngle(), motor2.shaft_velocity,sensor2.getVelocity(), pitch, voltage_control, target_pitch-pitch);
        // itter = 0;
        data_out.voltage = motor2.shaft_velocity;//voltage_control; 
        data_out.pitch = target_pitch+pitch;
        data_out.m_id = dir;
        sendMotorData(data_out);
      // }
  // }


  // read the tuning commands from Serial
  commander.run();
  // read the user command from bluetooth
  // handleBluetooth(bluetooth);
}

/**
  Function intended for connecting to the Gibalo application
  In one byte it receives either throttel or steering command
  - if received byte  in range [0,200] then throttle command in between [-100, 100]
  - if received byte  in range [210,250] then steering command in between [-20, 20]
*/
void handleBluetooth(Stream& bt_port) {
  int inByte;
  if (bt_port.available() > 0) {
    while (bt_port.available()) {
      inByte = bt_port.read();
    }
    inByte = inByte - 100;
    if (inByte == 155) {
      // ON Byte
      steering = 0;
      throttle = 0;
      state = 1;
      bt_port.println("State ON");
    } else if (inByte == 154) {
      // OFF Byte
      steering = 0;
      throttle = 0;
      state = 0;
      bt_port.println("State OFF");
    } else if (inByte >= -100 && inByte <= 100) {
      // throttle set-point Byte
      throttle = max_throttle *  ((float)inByte) / 100.0;
    } else if (inByte >= 110 && inByte <= 150) {
      // steering set-point Byte
      steering = max_steering * ((float)(inByte - 130.0)) / 20.0;
    } else {
      // Error Byte
      steering = 0;
      throttle = 0;
      bt_port.println("Error number!");
    }
    bt_port.flush();
  }
}