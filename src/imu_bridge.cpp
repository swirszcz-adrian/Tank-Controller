// ##### WARNING #####
// THIS NODE CAN ONLY BE RUN ON RASPBERRY PI!


#if !defined(__arm__) || defined(_WIN32)
/* 
 * The "if" clauzure above checks if device has ARM archtecture and if it isn't running on Windows
 * Although this is far from foolproof, it's a simple solution to check whether device has capability to actually run the code
 */


#include <iostream>


int main() {
    std::cout << "ERROR: This program can only be run on Raspberry Pi, preferably with Raspbian OS" << std::ebdl;

    return -1;
}



#else
/*
 * The code below will only be executed, if the device has ARM archtecture and isn't running on Windows
 */


#include "ros/ros.h"
#include "tank_controller/imu_utility.h"

#include <iostream>



int main() {
    imu::AdafruitBNO055 bno = imu::AdafruitBNO055();
    if (!bno.begin()) {
        return EXIT_FAILURE;
    }

    imu::delay(1000);
    int8_t temp = 0;
    bno.getTemperature(temp);
    std::cout << "Current temperature is: " << (int) temp << " C" << std::endl;
    
    bno.setExtCrystalUse(true);
    float vector[3];
    uint8_t system_status, gyro, accel, mag;
    while (true) {
        vector[0] = 0.0f;
        vector[1] = 0.0f;
        vector[2] = 0.0f;
        system_status = 0U;
        gyro = 0U;
        accel = 0U;
        mag = 0U;

        bno.getVector(vector, imu::AdafruitBNO055::VectorType::EULER);
        std::cout << "x=" << std::fixed << std::setw(9) << std::setprecision(2) << vector[0] 
                  << "; y=" << std::fixed << std::setw(9) << std::setprecision(2) << vector[1] 
                  << "; z=" << std::fixed << std::setw(9) << std::setprecision(2) << vector[2] <<";\n";

        bno.getCalibration(system_status, gyro, accel, mag);        
        std::cout << "sys_calib=" << (int) system_status
                  << "; gyro_calib=" << (int) gyro
                  << "; accel_calib=" << (int) accel
                  << "; mag_calib=" << (int) mag << ";\n";
        imu::delay(IMU_SAMPLERATE_DELAY_MS);
    }
}


#endif