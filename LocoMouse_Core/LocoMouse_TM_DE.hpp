/*	Defining Dennis' TM variation of LocoMouse.

A more general and flexible version of the TM algorithm.

Author: Joao Fayad (joaofayad@gmail.com)

Note: Adaptations to the TM variation were originally written by Dennis Eckmeier in MATLAB.

*/

#ifndef __LocoMouse_TM_DE_H_INCLUDED__
#define __LocoMouse_TM_DE_H_INCLUDED__

#include "LocoMouse_TM.hpp"

class LocoMouse_TM_DE : public LocoMouse_TM {
private:
	/*//Parameters that are specific to this method:
	cv::Mat DISK_FILTER; //Disk filter for filtering the image and compute BB

	//Other parameters for BB computation:
	double BOTTOM_THRESHOLD = 2.55, SIDE_THRESHOLD = 2.55; //0.01 in [0 255] range.
	int MIN_VIS_PIXEL = 1, MIN_PIXEL_COUNT = 500;

	void imfill(const Mat &Iin, Mat &Iout);*/

	int MIN_PIXEL_COUNT = 10;
	double WIDTH_MARGIN = 1.1;
	double SIDE_THRESHOLD = 255 * 0.05;

	//Computing BB for the TM setup:
	void computeMouseBox_DE(cv::Mat &I_side_view, double& bb_x);

	

public:
	LocoMouse_TM_DE(LocoMouse_ParseInputs INPUTS);

	using LocoMouse_TM::readFrame;

	//virtual void readFrame(); //Overloaded to perform post-processing steps

	virtual void computeBoundingBox(); //Overloaded to compute it in the new way.

};

#endif
