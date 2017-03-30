#include "LocoMouse_TM.hpp"
//LocoMouse_TM:
LocoMouse_TM::LocoMouse_TM(LocoMouse_ParseInputs INPUTS) : LocoMouse(INPUTS) {

	//Open file and initialize the disk filter:
	cv::FileStorage disk_filter(ref_path + "diskfilter.yml", cv::FileStorage::READ);
	if (!disk_filter.isOpened()) {
		throw std::invalid_argument("Failed to read config file: diskfilter.yml");
	}

	disk_filter["H"] >> DISK_FILTER;

	disk_filter.release();

	if (LM_PARAMS.LM_DEBUG) {
		DEBUG_TEXT << "Initialized a LocoMouse_TM class" << std::endl;
	}

	return;
}

void LocoMouse_TM::computeBoundingBox() {

	if (LM_PARAMS.LM_DEBUG) {
		DEBUG_TEXT << "===== Computing the bounding box coordinates: " << std::endl;
	}

	std::vector<double> bb_x(N_FRAMES);
	
	int y_bottom_pos = N_ROWS - 1; //Last pixel of the image.

	I = cv::Mat::zeros(N_ROWS, N_COLS, CV_8UC1);

	cv::Mat I_side_view = I(BB_SIDE_VIEW);


	if (LM_PARAMS.LM_DEBUG) {
		DEBUG_TEXT << "BB_SIDE_VIEW: " << BB_SIDE_VIEW << std::endl;
		DEBUG_TEXT << "I.size(): " << I.size() << std::endl;
		DEBUG_TEXT << "I_side_view.size(): " << I_side_view.size() << std::endl;
		DEBUG_TEXT << "----- Computing the bounding box position per frame: " << std::endl;

	}



	for (unsigned int i_frames = 0; i_frames < N_FRAMES; ++i_frames) {
		
		LocoMouse::readFrame(I); //Reading image using the base-class method.

		computeMouseBox_DD(I_side_view, bb_x[i_frames]);
		BB_Y_BOTTOM_POS[i_frames] = y_bottom_pos;
		BB_Y_SIDE_POS[i_frames] = 165 - 1; //164, but defined like this to facilitate comparison with the MATLAB code.
	}

	//Smoothing with moving average:
	vecmovingaverage(bb_x, BB_X_POS, LM_PARAMS.moving_average_window);

	//FIXME: Make sure these quantities are initialised to these values:
	//NOTE: By the definition of lims, the real dimensions should be [1]-[0] + 1. But to agree with the way MATLAB crops images, we drop the +1. This creates some inconsistencies as if the whole image is white, the width/height can never be the with/height of the image  
	BB_SIDE_MOUSE.width = 400;
	BB_SIDE_MOUSE.height = 150;
	
	BB_BOTTOM_MOUSE.width = 400;
	BB_BOTTOM_MOUSE.height = BB_BOTTOM_VIEW.height;
	return;
}


void LocoMouse_TM::computeMouseBox_DD(cv::Mat &I_SIDE, double& bb_x) {

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
	
	//FIXME: Mouse limits using Connected Components. Move all of this into a function for clarity. 
	Mat labels, stats, centroids;
	uint N_labels = connectedComponentsWithStats(I_side_binary, labels, stats, centroids, LM_PARAMS.conn_comp_connectivity, CV_32S);
	
	if (LM_PARAMS.LM_DEBUG)
		DEBUG_TEXT << "Number of connected components found: " << N_labels << "." << std::endl;


	if (N_labels > 1) {
		Mat I_temp;

		I_side_binary = Mat::zeros(labels.size(), CV_8UC1);
		//OpenCV assumes the first object to be the background (the largest object?).
		for (uint i_labels = 1; i_labels < N_labels; ++i_labels) {
			if (stats.at<uint>(i_labels, CC_STAT_AREA) >= MIN_PIXEL_COUNT) {
				compare(labels, i_labels, I_temp, CMP_EQ);
				I_side_binary = I_side_binary | I_temp;//Removing area if less than min_pixel_count
			}
		}
		I_side_binary = I_side_binary / 255;
		
		//Filter with the disk filter. To replicate results, load filter from MATLAB.
		filter2D(I_side_binary, I_side_binary, CV_8UC1, DISK_FILTER, Point(-1, -1), 0, BORDER_REPLICATE);
				
		//Perform imfill, which doesn't exist either.
		imfill(I_side_binary, I_side_binary);
		
		//Get the BB of the white region.
		cv::Mat Row_top;
		reduce(I_side_binary, Row_top, 0, CV_REDUCE_SUM, CV_32SC1);

		int lims_row_top[2];

		//Compute and assign values to bounding box:
		//FIXME: Handle the empty image case. Perform the check inside the function and return a bool. Check after each run if the result was valid.
		//Row_* marks x and with; Col_* marks y and height.
		firstLastOverT(Row_top, N_COLS, lims_row_top, LM_PARAMS.min_pixel_visible);
		

		//Assigning the values to the bounding box vectors:
		bb_x = (double) lims_row_top[1];

		if (LM_PARAMS.LM_DEBUG)
			DEBUG_TEXT << "Final bb_x location: " << bb_x << "." << std::endl;

	}
}

void LocoMouse_TM::readFrame() {

	LocoMouse::readFrame(I); //Read as in the base class.

	imadjust(I, I, 0, 0.6, 0, 1); //Adjusting brightness.

}


void LocoMouse_TM::imfill(const Mat &Iin, Mat &Iout) {
	//This function was written based on the code described in: http://www.learnopencv.com/filling-holes-in-an-image-using-opencv-python-c/

	bool debug = false; //FIXME: Implement proper debug flags

	// Floodfill from point(0, 0)
	Mat im_floodfill = Iin.clone();
	floodFill(im_floodfill, Point(0, 0), 255);

	// Invert floodfilled image
	Mat im_floodfill_inv;
	bitwise_not(im_floodfill, im_floodfill_inv);

	// Combine the two images to get the foreground.
	Mat Itemp;
	Itemp = (Iin | im_floodfill_inv);
	Itemp.copyTo(Iout);
}
