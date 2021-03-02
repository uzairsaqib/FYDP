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
const uint8_t segment_size = 150;

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

// Differential time used for displacement calculation (millisecond)
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

imu::Quaternion quat;

vector3D pos;
vector3D accel;

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

    // Calculate displacement (del_x = 0.5*acceleration*dt*dt)
    pos->x = 0.5*accel_data->x()*delta_time*delta_time;
    pos->y = 0.5*accel_data->y()*delta_time*delta_time;
    pos->z = 0.5*accel_data->z()*delta_time*delta_time;

    accel->x = accel_data->x();
    accel->y = accel_data->y();
    accel->z = accel_data->z();

//    // Calculate displacement (del_x = 0.5*acceleration*dt*dt) (cm)
//    pos->x = 0.5*accel_data->x()*100.0*delta_time*delta_time;
//    pos->y = 0.5*accel_data->y()*100.0*delta_time*delta_time;
//    pos->z = 0.5*accel_data->z()*100.0*delta_time*delta_time;
//
//    // Store acceleration data (cm/s^2)
//    accel->x = accel_data->x()*100.0;
//    accel->y = accel_data->y()*100.0;
//    accel->z = accel_data->z()*100.0;

    return true;
}

/*
 * bool conjugate(const imu::Quaternion * quat, imu::Quaternion * quat_conj)
 * 
 * Description: Transforms a quaternion q into its conjugate q*.
 * 
 * For quaternion q = (w, xi + yj + zk), its conjugate
 * q* = (w, -xi - yj - zk)
 * 
 * Return:
 * 
 * False: Input pointer(s) is NULL
 * True: Conjuagate quaternion q* of input quaternion q
 * 
 */
bool conjugate(const imu::Quaternion * quat, imu::Quaternion * quat_conj) {
  if (!quat || !quat_conj) {
    return false;
  }

  quat_conj.w() = quat->w();
  quat_conj.x() = -quat->x();
  quat_conj.y() = -quat->y();
  quat_conj.z() = -quat->z();

  return true;
}

///*
// * bool normalize(imu::Quaternion * quat)
// * 
// * Description: Transforms a quaternion into a unit quaternion 
// * normalized by its magnitude.
// * 
// * For quaternion q = (w, xi + yj + zk), its magnitude/norm 
// * |q| = sqrt(w*w + x*x + y*y + z*z)
// * 
// * Return:
// * 
// * False: Input pointer is NULL
// * True: Original quaternion transformed into unit quaternion
// * 
// */
//bool normalize(imu::Quaternion * quat) {
//  if (!quat) {
//    return false;  
//  }
//
//  int16_t w = quat->w();
//  int16_t x = quat->x();
//  int16_t y = quat->y();
//  int16_t z = quat->z();
//
//  float norm = sqrt(w*w + x*x + y*y + z*z);
//
//  quat->w() = round((float)quat->w()/norm);
//  quat->x() = round((float)quat->x()/norm);
//  quat->y() = round((float)quat->y()/norm);
//  quat->z() = round((float)quat->z()/norm);
//
//  return true;
//}

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
    
    // Delay by 1s to allow sensors to calibrate (idk I'm speculating here)
    delay(1000);
    forearm_imu.setExtCrystalUse(true);

    // Initialize shoulder IMU
    if (!shoulder_imu.begin()) 
    {
        Serial.println("BNO055 IMU on shoulder not detected");
        while(1);
    }

     
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
    quat = forearm_imu.getQuat();

    int16_t ww = quat.w()*quat.w();
    int16_t xx = quat.x()*quat.x();
    int16_t yy = quat.y()*quat.y();
    int16_t zz = quat.z()*quat.z();

    int16_t wx_2 = 2*quat.w()*quat.x();
    int16_t wy_2 = 2*quat.w()*quat.y();
    int16_t wz_2 = 2*quat.w()*quat.z();
    
    int16_t xy_2 = 2*quat.x()*quat.y();
    int16_t xz_2 = 2*quat.x()*quat.z();
    int16_t yz_2 = 2*quat.y()*quat.z();

    accel_imu->x() = accel_imu->x()*(ww + xx + yy + zz + xy_2 + wz_2 - wy_2 + xz_2);
    accel_imu->y() = accel_imu->y()*(xy_2 - wz_2 + ww - xx + yy - zz + wx_2 + yz_2);
    accel_imu->z() = accel_imu->z()*(wy_2 + xz_2 - wx_2 + yz_2 + ww - xx - yy + zz);
    
    // Populate forearm IMU position and acceleration data
    if (!populateDataIMU(&accel_imu, &pos, &accel)) {
      Serial.println("Function populateDataIMU failed for forearm_imu");
      return;
    }

    // Store IMU position and acceleration into forearm data array
    forearm_pos_vals[sample_num] = pos;
    forearm_accel_vals[sample_num] = accel;

    // Read shoulder IMU acceleration data
    accel_imu = shoulder_imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
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
            Serial.print(segment_num);
            Serial.print(",");
            
            // Loop through all sensor data arrays (6 in total)
            for (int data_idx = 0; data_idx < 6; data_idx++) {
                switch(data_idx) {
                    // 0, 1 indices contain pointers to EMG1 and EMG2 raw data
                    case 0:
                    case 1: {
                        unsigned int ** emg2_ptr = (unsigned int **)to_data[data_idx];
                        if (!emg2_ptr) {
                            // Serial.println("NULL emg_ptr returned from to_data[data_idx], data_idx = %d", data_idx);
                            return;
                        }
                        Serial.print((*emg2_ptr)[sample]);
                        Serial.print(",");
                        break;
                    }
                    // 2, 3, 4, 5 indices contain pointers to IMU1 and IMU2 displacement and acceleration data
                    case 2:
                    case 3:
                    case 4:
                    case 5: {
                        vector3D ** vector_ptr = (vector3D **)to_data[data_idx];
                        if (!vector_ptr) {
                            Serial.println("NULL vector_ptr returned from index 2");
                            return;
                        }
                        Serial.print((*vector_ptr)[sample].x);
                        Serial.print(",");
                        Serial.print((*vector_ptr)[sample].y);
                        Serial.print(",");
                        Serial.print((*vector_ptr)[sample].z);
                        Serial.print(",");
                        break;
                    }
                }
            }
            // Last element prints the tic detected value (prints all 0 for now, labelling will be done manually)
            Serial.println(0);
        }
        
        // Reset sample number to prepare for next data segment
        sample_num = 0;

        // All data collected for current segment, prepare next segment
        segment_num++;
    }

    // Busy wait until sufficient time has elapsed for next sample
    while ((micros() - t_start) < (bno055_samplerate_delay_ms * 1000)) {}
}
