#include "LocoMouse_Methods.hpp"
/* 
* When a new method is designed, it must be added here for LocoMouse to known which class to initialize. 
*/

std::unique_ptr<LocoMouse> LocoMouse_Initialize(LocoMouse_ParseInputs INPUT) {

	std::unique_ptr<LocoMouse> L;

	int mode = std::stoi(INPUT.METHOD);

	switch (mode) {
	case 0:
		L.reset(new LocoMouse(INPUT));
		break;
	case 1:
		L.reset(new LocoMouse_TM(INPUT));
		break;
	default:
		std::cout << "Unknown method option. Attempting to track with the default method." << std::endl;
		L.reset(new LocoMouse(INPUT));
	}

	return L;

}