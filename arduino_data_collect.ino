/* 
 * Header files
 */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/*
 * Analog Input Pins
 */

// Pin for EMG sensor attached to bicep
# define EMG_PIN1 A0

// Pin for EMG sensor attached to tricep
# define EMG_PIN2 A1

// Read IMU sensor data every 10ms (100Hz sampling frequency)
const uint8_t bno055_samplerate_delay_ms = 10;

// Temporal data segment size for clasification (in ms)
const uint8_t segment_size = 500;

// Global count for sample number per segment
uint8_t sample_num = 0;

// Running counter for total number of segments collected
uint8_t segment_num = 0;

// 25 samples per segment (Sample every 10ms for 250ms worth of temporal data)
// Sampling frequency is limited by 100Hz sampling rate of IMU
const uint8_t num_readings = segment_size/bno055_samplerate_delay_ms;

// EMG offset voltage = emg_supply_voltage/2
const float offset_voltage = 3.3/2.0;

// ADC offset (Arduino 10bit ADC = (2^10 - 1) = 1023 data points)
const uint16_t adc_offset = (offset_voltage/5.0)*1023;

// Differential time used for displacement calculation
const float delta_time = (float)(bno055_samplerate_delay_ms)/1000.0;

/*
 * IMU Data Containers
 * 
 * Declare one sensors_event_t objects to read acceleration 
 * data from IMU sensors
 * 
 * Declare two vector3D structs to store displacement and
 * acceleration data from IMU sensors
 * 
 */
typedef struct {
    float x;
    float y;
    float z;
} vector3D;

imu::Vector<3> accel_imu;

imu::Quaternion quat_forearm;
imu::Quaternion pure_quat;

vector3D pos;
vector3D accel;

// Forearm IMU Calibration Data
/*
16
-3
-21
-89
168
-207
0
-2
1
1000
950

 */

adafruit_bno055_offsets_t forearm_sensor_offsets = {
  .accel_offset_x = 16,
  .accel_offset_y = -3,
  .accel_offset_z = -21,
  
  .mag_offset_x = -89,
  .mag_offset_y = 168,
  .mag_offset_z = -207,

  .gyro_offset_x = 0,
  .gyro_offset_y = -2,
  .gyro_offset_z = 1,

  .accel_radius = 1000,
  .mag_radius = 950,
};

adafruit_bno055_offsets_t shoulder_sensor_offsets;

/*
 * Sensor Data Arrays
 */

// Raw analog values for bicep EMG (EMG_PIN1)
unsigned int * emg1_analog_vals;

// Raw analog values for tricep EMG (EMG_PIN2)
unsigned int * emg2_analog_vals;

// Calculated displacement values for forearm IMU
vector3D * forearm_pos_vals;

// Raw acceleration data for forearm IMU
vector3D * forearm_accel_vals;

// Calculated displacement values for shoulder IMU
vector3D * shoulder_pos_vals;

// Raw acceleration data for shoulder IMU
vector3D * shoulder_accel_vals;

/*
 * IMU Sensor Object Declaration
 * 
 * Adafruit_BNO055(
 *      int32_t sensorID = -1,                      // Sensor ID
 *      uint8_t address = BNO055_ADDRESS_A,         // I2C Device Address
 *      TwoWire *theWire = &Wire);                  // (idk)
 * 
 */

Adafruit_BNO055 forearm_imu = Adafruit_BNO055(1, BNO055_ADDRESS_A);
Adafruit_BNO055 shoulder_imu = Adafruit_BNO055(2, BNO055_ADDRESS_B);

/*
 * Array of pointers to relevant sensor data arrays
 */

void ** to_data[6] = {
    &emg1_analog_vals,                               // Index 0: EMG1 raw rectified data
    &emg2_analog_vals,                               // Index 1: EMG2 raw rectified data
    &forearm_pos_vals,                               // Index 2: Forearm IMU vector3D position data
    &shoulder_pos_vals,                              // Index 3: Shoulder IMU vector3D position data
    &forearm_accel_vals,                             // Index 4: Forearm IMU vector3D acceleration data
    &shoulder_accel_vals                             // Index 5: Shoulder IMU vector3D acceleration data
};

/*
 * Data Preprocessing Functions
 */

bool populateDataIMU(const imu::Vector<3> * accel_data, vector3D * pos, vector3D * accel) {
    // Check if any of the pointers are NULL
    if (!accel_data || !pos || !accel) {
        return false;
    }

    // Calculate displacement (del_x = 0.5*acceleration*dt*dt) (m)
    pos->x = 0.5*accel_data->x()*delta_time*delta_time;
    pos->y = 0.5*accel_data->y()*delta_time*delta_time;
    pos->z = 0.5*accel_data->z()*delta_time*delta_time;

    // Store acceleration data (m/s^2)
    accel->x = accel_data->x();
    accel->y = accel_data->y();
    accel->z = accel_data->z();

    return true;
}

/*
 * imu::Quaternion conjugateQuat(const imu::Quaternion &q)
 * 
 * Description: Transforms a quaternion q into its conjugate q*.
 * 
 * For quaternion q = (w, xi + yj + zk), its conjugate
 * q* = (w, -xi - yj - zk)
 * 
 * Return:
 * 
 * Conjuagate quaternion q* of input quaternion q
 * 
 */
imu::Quaternion conjugateQuat(const imu::Quaternion &q) {
  imu::Quaternion q_conj;

  q_conj.w() = q.w();
  q_conj.x() = -q.x();
  q_conj.y() = -q.y();
  q_conj.z() = -q.z();

  return q_conj;
}

/*
 * imu::Quaternion multiplyQuat(const imu::Quaternion &q1, const imu::Quaternion &q2)
 * 
 * Description: Multiplies two quaternions .
 * 
 * For quaternion q1 = (w, xi + yj + zk) and q2 = (a, bi + cj + dk), its product
 * q_p = ((w*a - x*b - y*c - z*d), (x*a + w*b + y*d - z*c)i, (y*a + w*c + z*b - x*d)j, (z*a + w*d + x*c - y*b)k)
 * 
 * Return:
 * 
 * Product quaternion q_p = q1*q2
 * 
 */
imu::Quaternion multiplyQuat(const imu::Quaternion &q1, const imu::Quaternion &q2) {
  imu::Quaternion q_p;

  q_p.w() = q1.w()*q2.w() - q1.x()*q2.x() - q1.y()*q2.y() - q1.z()*q2.z();
  q_p.x() = q1.x()*q2.w() + q1.w()*q2.x() + q1.y()*q2.z() - q1.z()*q2.y();
  q_p.y() = q1.y()*q2.w() + q1.w()*q2.y() + q1.z()*q2.x() - q1.x()*q2.z();
  q_p.z() = q1.z()*q2.w() + q1.w()*q2.z() + q1.x()*q2.y() - q1.y()*q2.x();

  return q_p;
}


void displayCalStatus(Adafruit_BNO055 &imu)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  imu.getCalibration(&system, &gyro, &accel, &mag);
 
  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }
 
  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}

void setup()
{
    // Begin serial interface at 9600 baud
    Serial.begin(9600);

    // Initialize forearm IMU
    if (!forearm_imu.begin()) 
    {
        Serial.println("BNO055 IMU on forearm not detected");
        while(1);
    }

    // Calibrate forearm IMU with preset data
    forearm_imu.setSensorOffsets(forearm_sensor_offsets);
    
    for (int i = 0; i < 20; i++) {
      displayCalStatus(forearm_imu);
    }
//    while (!forearm_imu.isFullyCalibrated()) {
//      displayCalStatus();
//    }
//    
//    Serial.println("FOREARM FULLY CALIBRATED");
//    forearm_imu.getSensorOffsets(sensor_offsets);
//    
//    Serial.println(sensor_offsets.accel_offset_x);
//    Serial.println(sensor_offsets.accel_offset_y);
//    Serial.println(sensor_offsets.accel_offset_z);
//    
//    Serial.println(sensor_offsets.mag_offset_x);
//    Serial.println(sensor_offsets.mag_offset_y);
//    Serial.println(sensor_offsets.mag_offset_z);
//
//    Serial.println(sensor_offsets.gyro_offset_x);
//    Serial.println(sensor_offsets.gyro_offset_y);
//    Serial.println(sensor_offsets.gyro_offset_z);
//
//    Serial.println(sensor_offsets.accel_radius);
//    Serial.println(sensor_offsets.mag_radius);
//    
    // Delay by 1s to allow sensors to calibrate (idk I'm speculating here)
    delay(1000);
    forearm_imu.setExtCrystalUse(true);

    // Initialize shoulder IMU
    if (!shoulder_imu.begin()) 
    {
        Serial.println("BNO055 IMU on shoulder not detected");
        while(1);
    }

    // Calibrate shoulder IMU with preset data
//    shoulder_imu.setSensorOffsets(sensor_offsets);
//    
//    Serial.println("SHOULDER IMU");
//    for (int i = 0; i < 20; i++) {
//      displayCalStatus(shoulder_imu);  
//    }

    while (!shoulder_imu.isFullyCalibrated()) {
      displayCalStatus(shoulder_imu);
    }
    
    Serial.println("SHOULDER FULLY CALIBRATED");
    shoulder_imu.getSensorOffsets(shoulder_sensor_offsets);
    
    Serial.println(shoulder_sensor_offsets.accel_offset_x);
    Serial.println(shoulder_sensor_offsets.accel_offset_y);
    Serial.println(shoulder_sensor_offsets.accel_offset_z);
    
    Serial.println(shoulder_sensor_offsets.mag_offset_x);
    Serial.println(shoulder_sensor_offsets.mag_offset_y);
    Serial.println(shoulder_sensor_offsets.mag_offset_z);

    Serial.println(shoulder_sensor_offsets.gyro_offset_x);
    Serial.println(shoulder_sensor_offsets.gyro_offset_y);
    Serial.println(shoulder_sensor_offsets.gyro_offset_z);

    Serial.println(shoulder_sensor_offsets.accel_radius);
    Serial.println(shoulder_sensor_offsets.mag_radius);

     
    // Delay by 1s to allow sensors to calibrate (idk I'm speculating here)
    delay(1000);
    shoulder_imu.setExtCrystalUse(true);

    // Allocate sensor data arrays
    emg1_analog_vals = (unsigned int *)malloc(sizeof(unsigned int) * num_readings);
    if (!emg1_analog_vals) {
        Serial.println("Failed to malloc emg1_analog_vals");
        while(1);
    }

    emg2_analog_vals = (unsigned int *)malloc(sizeof(unsigned int) * num_readings);
    if (!emg2_analog_vals) {
        Serial.println("Failed to malloc emg2_analog_vals");
        while(1);
    }
    
    forearm_pos_vals = (vector3D *)malloc(sizeof(vector3D) * num_readings);
    if (!forearm_pos_vals) {
        Serial.println("Failed to malloc forearm_pos_vals");
        while(1);
    }
    
    forearm_accel_vals = (vector3D *)malloc(sizeof(vector3D) * num_readings);
    if (!forearm_accel_vals) {
        Serial.println("Failed to malloc forearm_accel_vals");
        while(1);
    }
    
    shoulder_pos_vals = (vector3D *)malloc(sizeof(vector3D) * num_readings);
    if (!shoulder_pos_vals) {
        Serial.println("Failed to malloc shoulder_pos_vals");
        while(1);
    }
    
    shoulder_accel_vals = (vector3D *)malloc(sizeof(vector3D) * num_readings);
    if (!shoulder_accel_vals) {
        Serial.println("Failed to malloc shoulder_accel_vals");
        while(1);
    }

    Serial.print("Segment,");
    Serial.print("BicepEMG,");
    Serial.print("TricepEMG,");
    Serial.print("ForearmPosX,");
    Serial.print("ForearmPosY,");
    Serial.print("ForearmPosZ,");
    Serial.print("ShoulderPosX,");
    Serial.print("ShoulderPosY,");
    Serial.print("ShoulderPosZ,");
    Serial.print("ForearmAccelX,");
    Serial.print("ForearmAccelY,");
    Serial.print("ForearmAccelZ,");
    Serial.print("ShoulderAccelX,");
    Serial.print("ShoulderAccelY,");
    Serial.print("ShoulderAccelZ,");
    Serial.print("TicMotion");
}

void loop()
{
    // Start time (Used to implement sampling delay)
    unsigned long t_start = micros();

    // Read forearm IMU acceleration data
    accel_imu = forearm_imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    // Read forearm IMU absolute orientation data (quaternion output)
    quat_forearm = forearm_imu.getQuat();

    // Get conjugate of forearm quaternion
    imu::Quaternion quat_forearm_conj = conjugateQuat(quat_forearm);

    // Transform acceleration vector into pure quaternion p = (0, xi, yj, zk)
    pure_quat.w() = 0;
    pure_quat.x() = accel_imu.x();
    pure_quat.y() = accel_imu.y();
    pure_quat.z() = accel_imu.z();

    // Transform acceleration vector into global coordinate frame
    imu::Quaternion q_tmp = multiplyQuat(quat_forearm, pure_quat);
    
    imu::Quaternion pure_quat_transformed = multiplyQuat(q_tmp, quat_forearm_conj);

    float w = pure_quat_transformed.w();
    float x = pure_quat_transformed.x();
    float y = pure_quat_transformed.y();
    float z = pure_quat_transformed.z();

//    Serial.print("qW: ");
//    Serial.println(quat_forearm.w(), 4);
//    Serial.print("qX: ");
//    Serial.println(quat_forearm.x(), 4);
//    Serial.print("qY: ");
//    Serial.println(quat_forearm.y(), 4);
//    Serial.print("qZ: ");
//    Serial.println(quat_forearm.z(), 4);
//    

//    float ww = quat.w()*quat.w();
//    Serial.println(ww, 4);
//    float xx = quat.x()*quat.x();
//    Serial.println(xx, 4);
//    float yy = quat.y()*quat.y();
//    Serial.println(yy, 4);
//    float zz = quat.z()*quat.z();
//    Serial.println(zz, 4);

//    Serial.println(accel_imu.x(), 4);
//    Serial.println("END");
//    float wx_2 = 2*quat.w()*quat.x();
//    float wy_2 = 2*quat.w()*quat.y();
//    float wz_2 = 2*quat.w()*quat.z();
//    
//    float xy_2 = 2*quat.x()*quat.y();
//    float xz_2 = 2*quat.x()*quat.z();
//    float yz_2 = 2*quat.y()*quat.z();
//
//    float imu_x = accel_imu.x()*(ww + xx + yy + zz + xy_2 + wz_2 - wy_2 + xz_2);
//    float imu_y = accel_imu.y()*(xy_2 - wz_2 + ww - xx + yy - zz + wx_2 + yz_2);
//    float imu_z = accel_imu.z()*(wy_2 + xz_2 - wx_2 + yz_2 + ww - xx - yy + zz);

//    accel_imu.x() = accel_imu.x()*(ww + xx + yy + zz + xy_2 + wz_2 - wy_2 + xz_2);
//    accel_imu.y() = accel_imu.y()*(xy_2 - wz_2 + ww - xx + yy - zz + wx_2 + yz_2);
//    accel_imu.z() = accel_imu.z()*(wy_2 + xz_2 - wx_2 + yz_2 + ww - xx - yy + zz);
    
    // Populate forearm IMU position and acceleration data
    if (!populateDataIMU(&accel_imu, &pos, &accel)) {
      Serial.println("Function populateDataIMU failed for forearm_imu");
      return;
    }

//    Serial.print(accel.x);
//    Serial.print(",");
//    Serial.println(imu_x);

    // Store IMU position and acceleration into forearm data array
    forearm_pos_vals[sample_num] = pos;
    forearm_accel_vals[sample_num] = accel;

    // Read shoulder IMU acceleration data
    accel_imu = shoulder_imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    // Read forearm IMU absolute orientation data (quaternion output)
//    quat = shoulder_imu.getQuat();

//    ww = quat.w()*quat.w();
//    Serial.println(ww);
//    xx = quat.x()*quat.x();
//    Serial.println(xx);
//    yy = quat.y()*quat.y();
//    Serial.println(yy);
//    zz = quat.z()*quat.z();
//    Serial.println(zz);

//    wx_2 = 2*quat.w()*quat.x();
//    wy_2 = 2*quat.w()*quat.y();
//    wz_2 = 2*quat.w()*quat.z();
//    
//    xy_2 = 2*quat.x()*quat.y();
//    xz_2 = 2*quat.x()*quat.z();
//    yz_2 = 2*quat.y()*quat.z();
//
//    accel_imu.x() = accel_imu.x()*(ww + xx + yy + zz + xy_2 + wz_2 - wy_2 + xz_2);
//    accel_imu.y() = accel_imu.y()*(xy_2 - wz_2 + ww - xx + yy - zz + wx_2 + yz_2);
//    accel_imu.z() = accel_imu.z()*(wy_2 + xz_2 - wx_2 + yz_2 + ww - xx - yy + zz);
    
    // Populate shoulder IMU position and acceleration data
    if (!populateDataIMU(&accel_imu, &pos, &accel)) {
      Serial.println("Function populateDataIMU failed for shoulder_imu");
      return;
    }

    // Store IMU position and acceleration into shoulder data array
    shoulder_pos_vals[sample_num] = pos;
    shoulder_accel_vals[sample_num] = accel;

    // Read raw data from EMG sensors
    int emg1_no_offset = analogRead(EMG_PIN1) - adc_offset;
    int emg2_no_offset = analogRead(EMG_PIN2) - adc_offset;

    // Rectify raw EMG data (i.e. take absolute value)
    unsigned int emg1_rectified = abs(emg1_no_offset);
    unsigned int emg2_rectified = abs(emg2_no_offset);

    // Store rectified EMG data into respective data arrays
    emg1_analog_vals[sample_num] = emg1_rectified;
    emg2_analog_vals[sample_num] = emg2_rectified;

    sample_num++;

    // Check if number of samples has exceeded total allowable readings per segment (i.e. num_readings)
    if (sample_num >= num_readings)
    {
        // Print all sample data collected in current segment
        for (int sample = 0; sample < num_readings; sample++) {
            // First element prints the current segment number
//            Serial.print(segment_num);
//            Serial.print(",");
            
            // Loop through all sensor data arrays (6 in total)
            for (int data_idx = 0; data_idx < 6; data_idx++) {
                switch(data_idx) {
                    // 0, 1 indices contain pointers to EMG1 and EMG2 raw data
//                    case 0:
//                    case 1: {
//                        unsigned int ** emg2_ptr = (unsigned int **)to_data[data_idx];
//                        if (!emg2_ptr) {
//                            // Serial.println("NULL emg_ptr returned from to_data[data_idx], data_idx = %d", data_idx);
//                            return;
//                        }
//                        Serial.print((*emg2_ptr)[sample]);
//                        Serial.print(",");
//                        break;
//                    }
                    // 2, 3, 4, 5 indices contain pointers to IMU1 and IMU2 displacement and acceleration data
//                    case 2:
//                    case 3:
//                    case 4: {
//                    case 5: {
//                        vector3D ** vector_ptr = (vector3D **)to_data[data_idx];
//                        if (!vector_ptr) {
//                            Serial.println("NULL vector_ptr returned from index 2");
//                            return;
//                        }
//                        Serial.print((*vector_ptr)[sample].x); // Sitting down Aditya's desk facing wall, wall direction towards closet, moving IMU towards Aditya's dresser and humidifier (strafing motion)
//                        Serial.print(",");
//                        Serial.print((*vector_ptr)[sample].y); // Sitting down Aditya's desk facing wall, wall direction towards closet, moving IMU towards wall and door (back and forth motion)
//                        Serial.print(",");
//                        Serial.print((*vector_ptr)[sample].z); // Sitting down Aditya's desk facing wall, wall direction towards closet, moving IMU towards ceiling and floor (up and down motion)
//                        Serial.print(",");
//                        break;
//                    }
                }
            }
            // Last element prints the tic detected value (prints all 0 for now, labelling will be done manually)
//            Serial.println(0);
        }
        
        // Reset sample number to prepare for next data segment
        sample_num = 0;

        // All data collected for current segment, prepare next segment
        segment_num++;
    }

    // Busy wait until sufficient time has elapsed for next sample
    while ((micros() - t_start) < (bno055_samplerate_delay_ms * 1000)) {}
}
