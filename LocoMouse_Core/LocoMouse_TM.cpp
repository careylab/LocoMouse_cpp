#include "LocoMouse_TM.hpp"
//LocoMouse_TM:
LocoMouse_TM::LocoMouse_TM(LocoMouse_ParseInputs INPUTS) : LocoMouse(INPUTS) {

	//Open file and initialize the disk filter:
	cv::FileStorage disk_filter(ref_path + "diskfilter.yml", cv::FileStorage::READ);
	if (!disk_filter.isOpened()) {
		std::cout << ref_path + "diskfilter.yml" << std::endl;
		throw std::invalid_argument("Failed to read config file: diskfilter.yml");
	}

	disk_filter["H"] >> DISK_FILTER;

	disk_filter.release();

	//Loading algorithm specific parameters:
	LocoMouse_TM_Parameters(CONFIG_FILE);
	
	//FIXME: Create function instead of repeating block:
	if ((BB_SIDE_VIEW.width < ZERO_COL_PRE || BB_SIDE_VIEW.width < ZERO_COL_POST) & !LM_PARAMS.use_provided_bb) {
		
		std::cerr << "BB_SIDE_VIEW: " << BB_SIDE_VIEW << std::endl;
		std::cerr << "ZERO_COL_PRE, ZERO_COL_POST: " << ZERO_COL_PRE << " " << ZERO_COL_POST << endl;
		
		throw::std::invalid_argument("Side View image size is not compatible with the zero_col parameters for the bounding box computations. See the definition of the LocoMouse_TM class.");
	}

	if ((BB_SIDE_VIEW.height < ZERO_ROW_PRE || BB_SIDE_VIEW.height < ZERO_ROW_POST) & !LM_PARAMS.use_provided_bb) {

		std::cerr << "BB_SIDE_VIEW: " << BB_SIDE_VIEW << std::endl;
		std::cerr << "ZERO_ROW_PRE, ZERO_ROW_POST: " << ZERO_ROW_PRE << " " << ZERO_ROW_POST << endl;
		
		throw::std::invalid_argument("Side View image size is not compatible with the zero_row parameters for the bounding box computations. See the definition of the LocoMouse_TM class.");
	}


	if (LM_PARAMS.LM_DEBUG) {
		DEBUG_TEXT << "Initialized a LocoMouse_TM class" << std::endl;
	}

	return;
}

void LocoMouse_TM::LocoMouse_TM_Parameters(std::string config_file_name) {

	cv::FileStorage config_file(config_file_name, cv::FileStorage::READ);
	if (!config_file.isOpened()) {
		throw std::invalid_argument("Failed to read config file: " + config_file_name + ".");
	}

	int s_th, b_th, min_pixel_count, zcpre, zcpost, zrpre, zrpost, bbwith, bbheight_side;

	//In case there are exceptions:
	std::string error_message = "Invalid configuration parameter: ";

	//Other parameters for BB computation:
	config_file["bw_threshold_bottom"] >> b_th;

	if (b_th < 0 || b_th > 255) {
		error_message += "bw_threshold_bottom must belong to [0, 1].";
		throw std::invalid_argument(error_message);
	}
	BOTTOM_THRESHOLD = (unsigned int) b_th;
	
	config_file["bw_threshold_side"] >> s_th; //0.01 in [0 255] range.

	if (s_th < 0 || s_th > 255) {
		error_message += "bw_threshold_side must belong to [0, 255].";
		throw std::invalid_argument(error_message);
	}
	SIDE_THRESHOLD = (unsigned int)s_th;

	config_file["min_pixel_count"] >> min_pixel_count;
	
	
	if (min_pixel_count < 1) {
		error_message += "Min pixel count must be at least 1.";
		throw std::invalid_argument(error_message);
	}

	MIN_PIXEL_COUNT = (unsigned int) min_pixel_count;

	config_file["zero_col_pre"] >> zcpre;
	config_file["zero_col_post"] >> zcpost;
	config_file["zero_row_pre"] >> zrpre;
	config_file["zero_row_post"] >> zrpost;
	
	if (zcpost < 0 || zcpre < 0 || zrpost < 0 || zrpre < 0) {
		error_message += "zero_*_* parameters range from 0 to the relevant size of the image.";
		throw std::invalid_argument(error_message);
	}

	ZERO_COL_POST = (unsigned int)zcpost;
	ZERO_COL_PRE = (unsigned int)zcpre;
	ZERO_ROW_POST = (unsigned int)zrpost;
	ZERO_ROW_PRE = (unsigned int)zrpre;

	config_file["bb_width"] >> bbwith;

	if (bbwith < 1) {
		error_message += "bb_width must be at least 1 pixel.";
		throw std::invalid_argument(error_message);
	}
	BB_WIDTH = (unsigned int)bbwith;

	config_file["bb_height_side"] >> bbheight_side;

	if (bbheight_side < 1) {
		error_message += "bb_height_side must be at least 1 pixel.";
		throw std::invalid_argument(error_message);
	}
	BB_HEIGHT_SIDE = (unsigned int)bbheight_side;
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
	BB_SIDE_MOUSE.width = BB_WIDTH;
	BB_SIDE_MOUSE.height = BB_HEIGHT_SIDE;
	
	BB_BOTTOM_MOUSE.width = BB_WIDTH;
	BB_BOTTOM_MOUSE.height = BB_BOTTOM_VIEW.height;
	return;
}

cv::Mat LocoMouse_TM::bwAreaOpen(cv::Mat &Iin) {
	//Removes areas with less than MIN_PIXEL_COUNT
	
	cv::Mat labels, stats, centroids;
	uint N_labels = connectedComponentsWithStats(Iin, labels, stats, centroids, LM_PARAMS.conn_comp_connectivity, CV_32S);

	if (LM_PARAMS.LM_DEBUG)
		DEBUG_TEXT << "Number of connected components found: " << N_labels << "." << std::endl;

	cv::Mat Iout;
	if (N_labels > 1) {
		cv::Mat I_temp;

		Iout = Mat::zeros(labels.size(), CV_8UC1);

		//OpenCV assumes the first object to be the background (the largest object?).
		for (uint i_labels = 1; i_labels < N_labels; ++i_labels) {
			if (stats.at<uint>(i_labels, CC_STAT_AREA) >= MIN_PIXEL_COUNT) {
				compare(labels, i_labels, I_temp, CMP_EQ);
				Iout |= I_temp;//Removing area if less than min_pixel_count
			}
		}
		Iout = Iout / 255;
	}
	else {
		Iout = cv::Mat::zeros(Iin.size(), CV_8UC1);
	}

	return Iout;

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
	I_SIDE.colRange(0, ZERO_COL_PRE).setTo(0);
	I_SIDE.colRange(ZERO_COL_POST, N_COLS).setTo(0);
	I_SIDE.rowRange(0, ZERO_ROW_PRE).setTo(0);
	I_SIDE.rowRange(ZERO_ROW_POST, I_SIDE.rows).setTo(0);
		
	if (LM_PARAMS.LM_DEBUG)
		DEBUG_TEXT << "Setting regions to zero: Done. " << std::endl;

	Mat I_side_binary;
	threshold(I_SIDE, I_side_binary, SIDE_THRESHOLD, 1, 0);//MATLAB has the threshold as a percentage. So 1% is 2.55 out of 255 

	I_side_binary = bwAreaOpen(I_side_binary);
		
	//If no area we could skip remaining steps.

	filter2D(I_side_binary, I_side_binary, CV_8UC1, DISK_FILTER, Point(-1, -1), 0, BORDER_REPLICATE);

	//Perform imfill, which doesn't exist either.
	imfill(I_side_binary, I_side_binary);
	
	//Get the BB of the white region.
	cv::Mat Row_side;
	reduce(I_side_binary, Row_side, 0, CV_REDUCE_SUM, CV_32SC1);

	int lims_row_side[2];

	//Compute and assign values to bounding box:
	//FIXME: Handle the empty image case. Perform the check inside the function and return a bool. Check after each run if the result was valid.
	//Row_* marks x and with; Col_* marks y and height.
	firstLastOverT(Row_side, N_COLS, lims_row_side, LM_PARAMS.min_pixel_visible);

	//Assigning the values to the bounding box vectors:
	bb_x = (double)lims_row_side[1];
	
	if (LM_PARAMS.LM_DEBUG)
		DEBUG_TEXT << "Final bb_x location: " << bb_x << "." << std::endl;
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
