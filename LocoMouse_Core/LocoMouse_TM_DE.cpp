#include "LocoMouse_TM_DE.hpp"
//LocoMouse_TM_DE:
LocoMouse_TM_DE::LocoMouse_TM_DE(LocoMouse_ParseInputs INPUTS) : LocoMouse_TM(INPUTS) {
	//FIXME: Does this empty shell actually need to be implemented?
	return;
}

void LocoMouse_TM_DE::computeBoundingBox() {

	if (LM_PARAMS.LM_DEBUG) {
		DEBUG_TEXT << "===== Computing the bounding box coordinates: " << std::endl;
	}

	std::vector<double> bb_x(N_FRAMES);

	int y_bottom_pos = N_ROWS - 1; //Last pixel of the image.
	int y_side_pos = BB_SIDE_VIEW.height - 1; //Last pixel of the side view.

	I = cv::Mat::zeros(N_ROWS, N_COLS, CV_8UC1);

	cv::Mat I_bottom_view = I(BB_BOTTOM_VIEW);
	cv::Mat I_side_view = I(BB_SIDE_VIEW);
	
	if (LM_PARAMS.LM_DEBUG) {
		DEBUG_TEXT << "BB_SIDE_VIEW: " << BB_SIDE_VIEW << std::endl;
		DEBUG_TEXT << "I.size(): " << I.size() << std::endl;
		DEBUG_TEXT << "I_side_view.size(): " << I_side_view.size() << std::endl;
		DEBUG_TEXT << "----- Computing the bounding box position per frame: " << std::endl;

	}
	
	for (unsigned int i_frames = 0; i_frames < N_FRAMES; ++i_frames) {

		LocoMouse::readFrame(I); //Reading image using the base-class method.

		computeMouseBox_DE(I_side_view, bb_x[i_frames]);
		
		BB_Y_BOTTOM_POS[i_frames] = y_bottom_pos;
		BB_Y_SIDE_POS[i_frames] = y_side_pos;
	}

	//Smoothing with moving average:
	vecmovingaverage(bb_x, BB_X_POS, LM_PARAMS.moving_average_window);

	//FIXME: Make sure these quantities are initialised to these values:
	//NOTE: By the definition of lims, the real dimensions should be [1]-[0] + 1. But to agree with the way MATLAB crops images, we drop the +1. This creates some inconsistencies as if the whole image is white, the width/height can never be the with/height of the image  
	BB_SIDE_MOUSE.width = 400;
	BB_SIDE_MOUSE.height = BB_SIDE_VIEW.height;

	BB_BOTTOM_MOUSE.width = 400;
	BB_BOTTOM_MOUSE.height = BB_BOTTOM_VIEW.height;
	return;
}


void LocoMouse_TM_DE::computeMouseBox_DE(cv::Mat &I_SIDE, double& bb_x) {

	//Reading next frame:
	if (LM_PARAMS.LM_DEBUG)
		DEBUG_TEXT << "=== computeMouseBox_DD: " << std::endl;

	//NOTE: As we are using this just on the background part, it is ok to modify It directly.
	imadjust_default(I_SIDE, I_SIDE); //Adjusting constrast by saturating 1% of the data.

	if (LM_PARAMS.LM_DEBUG)
		DEBUG_TEXT << "imadjust_default_parameters: Done. " << std::endl;

	//These really are hand-set like this for the TM:
	I_SIDE.colRange(0, 46).setTo(0);
	I_SIDE.colRange(760, N_COLS).setTo(0);
	I_SIDE.rowRange(0, 100).setTo(0);
	I_SIDE.rowRange(149, I_SIDE.rows).setTo(0);

	if (LM_PARAMS.LM_DEBUG)
		DEBUG_TEXT << "Setting regions to zero: Done. " << std::endl;

	Mat I_side_binary;
	threshold(I_SIDE, I_side_binary, SIDE_THRESHOLD, 1, 0);//MATLAB has the threshold as a percentage. So 1% is 2.55 out of 255 
	
	//I_side_binary = bwAreaOpen(I_side_binary);

		//Filter with the disk filter. To replicate results, load filter from MATLAB.
	//filter2D(I_side_binary, I_side_binary, CV_8UC1, DISK_FILTER, Point(-1, -1), 0, BORDER_REPLICATE);

	//Perform imfill, which doesn't exist either.
	//imfill(I_side_binary, I_side_binary);

	//Search for the largest object: encapsulate inside a funciton too.
	//selectLargestRegion(I_side_binary, I_side_binary);
	
	//Get the limits of the largest object:
	cv::Mat Row_side;
	cv::reduce(I_side_binary, Row_side, 0, CV_REDUCE_SUM, CV_32FC1);

	int lims_row_side[2];

	//Compute and assign values to bounding box:
	//FIXME: Handle the empty image case. Perform the check inside the function and return a bool. Check after each run if the result was valid.
	//Row_* marks x and with; Col_* marks y and height.
	firstLastOverT(Row_side, N_COLS, lims_row_side, MIN_PIXEL_COUNT);


	//Assigning the values to the bounding box vectors:
	//if (lims_row_side[1] < 0) {
	//	std::cout << CURRENT_FRAME << " " << lims_row_side[0] << " " << lims_row_side[1] << std::endl;
	//}

	bb_x = std::min((double) (BB_SIDE_VIEW.width-1), (double) lims_row_side[1] * WIDTH_MARGIN);

	if (LM_PARAMS.LM_DEBUG)
		DEBUG_TEXT << "Final bb_x location: " << bb_x << "." << std::endl;

}