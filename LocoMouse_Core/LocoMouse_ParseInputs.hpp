/*	Interface between the console inputs and the LocoMouse algorithm.

Author: Joao Fayad (joaofayad@gmail.com)

*/

#ifndef __LocoMouse_ParseInputs_H_INCLUDED__
#define __LocoMouse_ParseInputs_H_INCLUDED__
#include <iostream>
#include <string>

#ifdef __linux__

#include <libgen.h>

#endif


class LocoMouse_ParseInputs {
public:
	std::string LM_CALL, CONFIG_FILE, VIDEO_FILE, MODEL_FILE, BKG_FILE, CALIBRATION_FILE, FLIP_CHAR, OUTPUT_PATH, METHOD, REF_PATH, FILE_STEM;

	LocoMouse_ParseInputs(int argc, char* argv[]);
	void configurePath(char* lm_call);
	std::string stripFileName(const std::string &s);

};

#endif
#pragma once
