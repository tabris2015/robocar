#include <iostream>
#include "VL53L0X.hpp"

int main() {
	VL53L0X sensor;
	sensor.initialize();
	sensor.setTimeout(200);

	for (int i = 0; i < 1000; ++i) {
		uint16_t distance = sensor.readRangeSingleMillimeters();
		if (sensor.timeoutOccurred()) {
			std::cout << "timeout!" << std::endl;
		} else {
			std::cout << distance << std::endl;
		}
	}

	return 0;
}
