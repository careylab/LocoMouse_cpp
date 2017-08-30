/*	Defining the TM variation of LocoMouse.

The TM version of LocoMouse is defined as inheriting the default LocoMouse algorithm and modifying some methods.

Author: Joao Fayad (joaofayad@gmail.com)

Note: The TM adaptations were originally developed by Dana Darmohrray in MATLAB.

*/

#ifndef __LocoMouse_TM_H_INCLUDED__
#define __LocoMouse_TM_H_INCLUDED__

#include "LocoMouse_class.hpp"

class LocoMouse_TM : public LocoMouse {
protected:
	//Parameters that are specific to this method:
	cv::Mat DISK_FILTER; //Disk filter for filtering the image and compute BB

	//Other parameters for BB computation:
	double BOTTOM_THRESHOLD = 2.55, SIDE_THRESHOLD = 2.55; //0.01 in [0 255] range.
	unsigned int MIN_PIXEL_COUNT = 500;
	
	//Zeroing out cols and rows for background subtraction: These parameters are hardcoded and specific to the setups where these methods were developed.
	unsigned int ZERO_COL_PRE = 46, ZERO_COL_POST = 760;
	unsigned int ZERO_ROW_PRE = 100, ZERO_ROW_POST = 149;

	unsigned int BB_WIDTH = 400;
	unsigned int BB_HEIGHT_SIDE = 150;

	void imfill(const Mat &Iin, Mat &Iout);

	cv::Mat bwAreaOpen(cv::Mat &Iin);

private:
	//Computing BB for the TM setup:
	void computeMouseBox_DD(cv::Mat &I_side_view, double& bb_x);

	void LocoMouse_TM_Parameters(std::string config_file_name);

public:
	LocoMouse_TM(LocoMouse_ParseInputs INPUTS);
	
	virtual void readFrame(); //Overloaded to perform post-processing steps
	
	virtual void computeBoundingBox(); //Overloaded to compute it in the new way.

};

#endif
