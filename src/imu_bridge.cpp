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
    std::cout << "Current temperature is: " << temp << " C" << std::endl;
    
    bno.setExtCrystalUse(true);
    float vector[3];
    while (true) {
        vector[0] = 0.0f;
        vector[1] = 0.0f;
        vector[2] = 0.0f;
        bno.getVector(vector, imu::AdafruitBNO055::VectorType::EULER);
        std::cout << "x=" << vector[0] << "; y=" << vector[1] << "; z=" << vector[2] <<";\n";
        imu::delay(IMU_SAMPLERATE_DELAY_MS);
    }
}


#endif