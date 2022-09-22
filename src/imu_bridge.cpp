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
#include "tank_controller/i2c_bno055.h"

#include <iostream>


int main() {
    bno055::ConnectionBridge bno = {};
    int temp = 0;
    bno.getTemperature(temp);
    std::cout << "Current temperature is: " << temp << " C" << std::endl;
    bno.setExtCrystalUse(true);
    bool read_success = false;
    float vector[3] = {};
    while (true) {
        read_success = bno.getVector(vector, bno055::VectorMappings::VECTOR_EULER);
        std::cout << "x=" << std::fixed << std::setw(9) << std::setprecision(2) << vector[0] 
                  << "; y=" << std::fixed << std::setw(9) << std::setprecision(2) << vector[1] 
                  << "; z=" << std::fixed << std::setw(9) << std::setprecision(2) << vector[2] <<";\n";
        bno055::delay(100);
    }
}


#endif