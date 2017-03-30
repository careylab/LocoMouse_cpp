/*	List of different methods to support in the LocoMouse system.

Every new modification of the LocoMouse system should be implemented as a class that derives from the LocoMouse
class and overloads existing methods.

New classes must be added here for inclusion on the main program:
- Make sure methods to overload are defined as virtual in LocoMouse_class.hpp
- Add the new class' .h file
- Add the initialization option for the new class on LocoMouse_Initialize (LocoMouse_Methods.cpp) 

Author: Joao Fayad (joaofayad@gmail.com)

*/

#ifndef __LocoMouse_Methods_H_INCLUDED__
#define __LocoMouse_Methods_H_INCLUDED__
#include "LocoMouse_ParseInputs.hpp"
#include "LocoMouse_class.hpp" //The standard class

//Include new modifications:
#include "LocoMouse_TM.hpp"

std::unique_ptr<LocoMouse> LocoMouse_Initialize(LocoMouse_ParseInputs INPUTS);

#endif
