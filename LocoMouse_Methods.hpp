/*	List of different methods to support in the LocoMouse system.

Every new modification of the LocoMouse system should be implemented as a class that derives from the LocoMouse
class and overloads existing methods.

New classes must be added here for inclusion on the main program:
- Add the new class' .h file
- Add the initialization option for the new class base on the FIXME parameter of the input yml.config file:

Author: Joao Fayad (joaofayad@gmail.com)



*/

#ifndef __LocoMouse_Methods_H_INCLUDED__
#define __LocoMouse_Methods_H_INCLUDED__
#include "LocoMouse_ParseInputs.hpp"
#include "LocoMouse_class.hpp" //The stadard class

//Include new modifications:
#include "LocoMouse_TM.hpp"
//#include "LocoMouse_HF.hpp"
//#include "LocoMouse_RotaryTM.hpp


std::unique_ptr<LocoMouse> LocoMouse_Initialize(LocoMouse_ParseInputs INPUTS);

#endif
