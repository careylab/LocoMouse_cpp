#include "LocoMouse_class.hpp"

//----- LocoMouse_Parameters
LocoMouse_Parameters::LocoMouse_Parameters(std::string config_file_name) {

	cv::FileStorage config_file(config_file_name, cv::FileStorage::READ);
	if (!config_file.isOpened()) {
		std::cout << "Failed to read config file!" << std::endl;
		//FIXME: Throw exception and handle it properly!
	}

	int lm_debug;
	config_file["debug"] >> lm_debug;
	LM_DEBUG = lm_debug > 0;

	//Connectivity for the Connected components algorithm.
	config_file["conn_comp_connectivity"] >> conn_comp_connectivity;

	if ((conn_comp_connectivity != 4) && (conn_comp_connectivity != 8)) {
		std::cout << "conn_comp_connectivity must be either 4 or 8!" << std::endl;
		////return -1;
	}

	//Median Filter used for bb estimation:
	config_file["median_filter_size"] >> median_filter_size;
	if (median_filter_size % 2 == 0) {
		std::cout << "median_filter_size must be odd." << std::endl;
		////return -2;
	}

	config_file["min_pixel_visible"] >> min_pixel_visible; //Minimum number of visible pixels to consider a valid BB boundary.
	if (min_pixel_visible < 0) {
		std::cout << "min_pixel_visible must be non-negative." << std::endl;
		//return -3;
	}

	config_file["top_bottom_min_overlap"] >> top_bottom_min_overlap; // Boxes must overlap for at least 70% for matches to be considered valid.
	if ((top_bottom_min_overlap < 0) || (top_bottom_min_overlap > 1)) {
		std::cout << "top_bottom_min_overlap must belong to [0,1]." << std::endl;
		//return -4;
	}

	config_file["maximum_normalized_beacon_distance"] >> maximum_normalized_beacon_distance; //Candidates with normalized distance lower than this are suppressed.
	if ((maximum_normalized_beacon_distance < 0) || (maximum_normalized_beacon_distance > 1)) {
		std::cout << "maximum_normalized_beacon_distance must belong to [0,1]." << std::endl;
		//return -5;
	}
	//FIXME: These two parameters are set in pixels and they depend on the framerate and resolution of the video. Calibrate with the training videos and set them as percentages of the bounding box dimension for a generalized behaviour.
	config_file["max_displacement"] >> max_displacement; //Maximum displacement in pixels allowed between two frames (There is a velocity component added to this).
	if (max_displacement < 0) {
		std::cout << "max_displacement must be non-negative." << std::endl;
		//return -6;
	}

	config_file["max_displacement_top"] >> max_displacement_top;
	if (max_displacement_top < 0) {
		std::cout << "max_displacement_top must be non-negative." << std::endl;
		//return -7;
	}

	config_file["occlusion_grid_spacing_pixels_top"] >> occlusion_grid_spacing_pixels_top;
	if (occlusion_grid_spacing_pixels_top < 0) {
		std::cout << "occlusion_grid_spacing_pixels_top must be non-negative." << std::endl;
		//return -8;
	}

	config_file["occlusion_grid_spacing_pixels"] >> occlusion_grid_spacing_pixels; //Spacing of the occlusion grid in pixels, highly dependent on resolution.
	if (occlusion_grid_spacing_pixels < 0) {
		std::cout << "occlusion_grid_spacing_pixels must be non-negative." << std::endl;
		//return -9;
	}

	config_file["body_proportion_bounding_box"] >> body_proportion_bounding_box; //The occlusion grid extends for only this percentage of the BB on the right side.
	if ((body_proportion_bounding_box < 0) || (body_proportion_bounding_box > 1)) {
		std::cout << "body_proportion_bounding_box must be non-negative." << std::endl;
		//return -10;
	}

	config_file["tail_sub_bounding_box"] >> tail_sub_bounding_box; //The percentage counting from the left of the box where the tail is looked for.
	if ((tail_sub_bounding_box < 0) || (tail_sub_bounding_box > 1)) {
		std::cout << "tail_sub_bounding_box must be non-negative." << std::endl;
		//return -11;
	}

	config_file["alpha_vel"] >> alpha_vel; //Relative term for the velocity costs on the match2nd tracking algorithm.
	if (alpha_vel < 0) {
		std::cout << "alpha_vel must be non-negative." << std::endl;
		//return -12;
	}

	config_file["alpha_vel_top"] >> alpha_vel_top;
	if (alpha_vel_top < 0) {
		std::cout << "alpha_vel_top must be non-negative." << std::endl;
		//return -13;
	}

	config_file["pairwise_occluded_cost"] >> pairwise_occluded_cost;//Cost for moving to and between occluded points on the match2nd algorithm.
	if (alpha_vel_top < 0) {
		std::cout << "pairwise_occluded_cost must be non-negative." << std::endl;
		//return -14;
	}

	config_file["moving_average_window"] >> moving_average_window; //Must be odd.
	if (moving_average_window % 2 == 0) {
		std::cout << "moving_average_window must be odd." << std::endl;
		//return -15;
	}

	config_file["mode"] >> mode_int;
	if (mode_int > 2 || mode_int < 0) {
		std::cout << "mode must belong to {0,1,2}." << std::endl;
		//return -16;
	}

	cv::Mat W;
	config_file["location_prior"] >> W;
	if ((W.cols != 7) || (W.rows != 5)) {
		std::cout << "location_prior must be a 5x7 matrix." << std::endl;
		//return -17;
	}

	for (int i_features = 0; i_features < N_paws; ++i_features) {
		double* Wptr = W.ptr<double>(i_features);
		PRIOR_PAW.push_back(LocoMouse_LocationPrior(Wptr[0], Wptr[1], Wptr[2], Wptr[3], Wptr[4], Wptr[5], Wptr[6]));
	}
	double* Wptr = W.ptr<double>(N_paws);
	PRIOR_SNOUT.push_back(LocoMouse_LocationPrior(Wptr[0], Wptr[1], Wptr[2], Wptr[3], Wptr[4], Wptr[5], Wptr[6]));

	config_file["use_reference_image"] >> use_reference_image;

	//FIXME: Check what is the best way to define the reference image/histogram given that it comes from different methods.
	/*if (use_reference_image) {

		std::string ref_name;
		config_file["reference_image_path"] >> ref_name;

		std::string ref_image_path = REF_PATH + "/" + ref_name;

		cv::Mat Iref = cv::imread(ref_image_path);
		if (!Iref.data) {
			std::cout << "Failed to read reference image." << std::endl;
			//return -18;
		}

		//Computing the normalized histogram:
		cv::Mat hist;
		int channels[] = { 0 };
		int histSize[] = { 256 };
		float range[] = { 0, 256 };
		const float* ranges[] = { range };

		calcHist(&Iref, 1, channels, cv::Mat(), // do not use mask
			hist, 1, histSize, ranges,
			true, // the histogram is uniform
			false);

		hist = hist / (Iref.cols*Iref.rows);

		//Computing the CDF:
		//FIXME: Find a better way of checking this: CV_Assert(reference_cdf.rows == 1);
		float* pcumsum = reference_cdf.ptr<float>(0);

		pcumsum[0] = hist.at<float>(0);
		for (int i = 1; i < reference_cdf.cols; i++) {
			pcumsum[i] = pcumsum[i - 1] + hist.at<float>(i);
		}

		std::cout << std::endl;

	}*/

	config_file["use_provided_bounding_box"] >> user_provided_bb;

	/*if (user_provided_bb) {
		//FIXME: Write this properly with exceptions
		cv::Mat T, B;
		config_file["bounding_box_top"] >> T;
		if (T.rows != 1 | T.cols != 4) {
			std::cout << "bounding_box_top must be a 1x4 opencv matrix." << std::endl;
			//return -1;
		}
		if (T.type() != CV_32SC1) {
			std::cout << "bounding_box_top must be provided as a 1x4 opencv matrix of type int (yml: i)." << std::endl;
			//return -1;
		}

		BB_user_top = cv::Rect(T.at<int>(0, 0), T.at<int>(0, 1), T.at<int>(0, 2), T.at<int>(0, 3));

		//if (BB_user_top.x < 0 | BB_user_top.y < 0 | (BB_user_top.x + BB_user_top.width) >= N_cols | (BB_user_top.y + BB_user_top.height)  >= N_rows) {
		//std::cout << "bounding_box_top exceeds input image dimensions!" << std::endl;
		///std::cout << BB_user_top << std::endl;
		//std::cout << "Image size (h,w): " << N_rows << " " << N_cols << std::endl;
		//return -1;
		//}

		config_file["bounding_box_bottom"] >> B;
		if (B.rows != 1 | B.cols != 4) {
			std::cout << "bounding_box_bottom must be a 1x4 opencv matrix." << std::endl;
			//return -1;
		}
		if (B.type() != CV_32SC1) {
			std::cout << "bounding_box_top must be provided as a 1x4 opencv matrix of type in (yml: i)." << std::endl;
			//return -1;
		}

		BB_user_bottom = cv::Rect(B.at<int>(0, 0), B.at<int>(0, 1), B.at<int>(0, 2), B.at<int>(0, 3));

		//if (BB_user_bottom.x < 0 | BB_user_bottom.y < 0 | (BB_user_bottom.x + BB_user_bottom.width) >= N_cols | (BB_user_bottom.y + BB_user_bottom.height) >= N_rows) {
		//std::cout << "bounding_box_bottom exceeds input image dimensions!" << std::endl;
		//std::cout << BB_user_bottom << std::endl;
		//std::cout << "Image size (h,w): " << N_rows << " " << N_cols << std::endl;
		////return -1;
		//}
	}*/

	config_file.release();
	//return 0;
}

LocoMouse_Parameters::LocoMouse_Parameters() {

}

LocoMouse_Parameters& LocoMouse_Parameters::operator=(LocoMouse_Parameters &&other) {

	//User configurable via config.yml:
	conn_comp_connectivity = other.conn_comp_connectivity;
	median_filter_size = other.median_filter_size;
	min_pixel_visible = other.min_pixel_visible;
	top_bottom_min_overlap = other.top_bottom_min_overlap;
	maximum_normalized_beacon_distance = other.maximum_normalized_beacon_distance;

	max_displacement = other.max_displacement;
	max_displacement_top = other.max_displacement_top;
	occlusion_grid_spacing_pixels_top = other.occlusion_grid_spacing_pixels_top;
	occlusion_grid_spacing_pixels = other.occlusion_grid_spacing_pixels;

	body_proportion_bounding_box = other.body_proportion_bounding_box;
	tail_sub_bounding_box = other.tail_sub_bounding_box;
	alpha_vel = other.alpha_vel;
	alpha_vel_top = other.alpha_vel_top;
	pairwise_occluded_cost = other.pairwise_occluded_cost;
	moving_average_window = other.moving_average_window;

	mode_int = other.mode_int;
	use_reference_image = other.use_reference_image;
	user_provided_bb = other.user_provided_bb;

	//To read from inputs:
	REF_PATH = other.REF_PATH;
	LM_DEBUG = other.LM_DEBUG;
	LM_FRAME_TO_DEBUG = other.LM_FRAME_TO_DEBUG;

	for (unsigned int i_paws = 0; i_paws < N_paws; ++i_paws) {
		PRIOR_PAW.push_back(other.PRIOR_PAW[i_paws]);
	}

	PRIOR_SNOUT.push_back(other.PRIOR_SNOUT[0]);

	return *this;
};



//----- LocoMouse:
LocoMouse::LocoMouse(int argc, char* argv[]) {

	std::cout << "Using OpenCV version: " << CV_VERSION << std::endl;

	if (argc != 8) {
		//FIXME: Find out how to stop the code from progressing.
	
		std::cout << "Warning: Invalid input list. The input should be: LocoMouse config.yml video.avi background.png model_file.yml calibration_file.yml side_char output_folder." << std::endl;
		std::cout << "Attempting to run with default paramters..." << std::endl;

		LM_CALL = std::string(argv[0]);
		VIDEO_FILE = "temp.avi";

		configurePath();

		CONFIG_FILE = ref_path + "config.yml";
		VIDEO_FILE = ref_path + "L7Y9_control1_L.avi";
		BKG_FILE = ref_path + "L7Y9_control1_L.png";
		MODEL_FILE = ref_path + "model_LocoMouse_paper.yml";
		CALIBRATION_FILE = ref_path + "IDX_pen_correct_fields2.yml";
		FLIP_CHAR = "L";
		OUTPUT_PATH = ref_path + ".";
		
	}
	else {
		LM_CALL = std::string(argv[0]);
		CONFIG_FILE = std::string(argv[1]);
		VIDEO_FILE = std::string(argv[2]);
		MODEL_FILE = std::string(argv[4]);
		BKG_FILE = std::string(argv[3]);
		CALIBRATION_FILE = std::string(argv[5]);
		FLIP_CHAR = std::string(argv[6]);
		OUTPUT_PATH = std::string(argv[7]);
	}

	//FIXME: Throw exceptions if any of these fail!
	LM_PARAMS = LocoMouse_Parameters(CONFIG_FILE);
	loadVideo();
	loadBackground();
	loadCalibration();

	validateImageVideoSize();

	M = LocoMouse_Model(MODEL_FILE);

	loadFlip();

	configurePath();

	


	//Initializing other variables that depend on N_FRAMES
	BB_X_POS.reserve(N_FRAMES);
	BB_Y_BOTTOM_POS.reserve(N_FRAMES);
	BB_Y_SIDE_POS.reserve(N_FRAMES);

	UNARY_BOTTOM_PAW.reserve(N_FRAMES);
	UNARY_BOTTOM_SNOUT.reserve(N_FRAMES);

	OUTPUT = cv::FileStorage(output_file, cv::FileStorage::WRITE);

	if (!OUTPUT.isOpened()) {
		throw std::runtime_error(Stream_Formatter() << "Could not create output file:  " << output_file << "\n");
	}

	if (LM_PARAMS.LM_DEBUG) {
		DEBUG_OUTPUT = cv::FileStorage(debug_file, cv::FileStorage::WRITE);

		if (!DEBUG_OUTPUT.isOpened()) {
			throw std::runtime_error(Stream_Formatter() << "Could not create debug file: " << debug_file << "\n");
			//FIXME: Throw exception.
		}

		DEBUG_TEXT.open(debug_text);

	}
	
	//FIXME: Check how to deal with the need for this file. Hardcode it?
	//		//Behaviour mode:
	//		//cv::Mat H;
	//		if (mode_int == 1) {
	//
	//			std::string disk_path;
	//
	//#ifdef _WIN32
	//			_makepath_s(path_buffer, drive, dir, "diskfilter", "yml"); // Creating path for diskfilter.yml at the same folder as LocoMouse.exe
	//			disk_path = std::string(path_buffer);
	//#elif __linux__
	//			disk_path = ref_path + "/diskfilter.yml";
	//#endif
	//			FileStorage disk_filter(ref_path, FileStorage::READ);
	//			if (!disk_filter.isOpened()) {
	//				cout << "Could not load the diskfilter.yml at " << disk_path << ". This is essential for the T mode." << std::endl;
	//				//return -1;
	//			}
	//			disk_filter["H"] >> H;
	//			disk_filter.release();
	//			
	//			if (LM_PARAMS.LM_DEBUG) {
	//				DEBUG_TEXT << "Disk filter loaded." << std::endl;
	//			}
	//		}
}

void LocoMouse::loadVideo() {

	V = cv::VideoCapture(VIDEO_FILE);

	if (!V.isOpened()) {
		std::cout << "Error: Could not open the video file." << std::endl;
		//FIXME: Find out how to throw an error here.
	}

	//FIXME: So far opencv has this option but it doesn't work. conversion has to be done manually.
	//V.set(CV_CAP_PROP_CONVERT_RGB, false);
	//V.set(CAP_PROP_FORMAT, CV_8UC1);

	//Getting Video frames:
	if (LM_PARAMS.LM_DEBUG) {
		const uint N_frames_video = V.get(cv::CAP_PROP_FRAME_COUNT);
		N_FRAMES = (N_frames_video < N_FRAMES_DEBUG) ? N_frames_video : N_FRAMES_DEBUG;
	}
	else {
		N_FRAMES = V.get(cv::CAP_PROP_FRAME_COUNT);
	}

	if (N_FRAMES < 1) {
		std::cout << "Error: Video has no images to read from." << std::endl;
		//FIXME: Find out how to throw an error here.
	}

	return;
}

void LocoMouse::loadBackground() {

	//Reading the background image:

	BKG = cv::imread(BKG_FILE, CV_LOAD_IMAGE_GRAYSCALE);
	if (!BKG.data) {
		std::cout << "Error: Could not open the background image." << std::endl;
		//FIXME: Find out how to throw an error here.
	}

	if (LM_PARAMS.LM_DEBUG) {
		DEBUG_TEXT << "Background image size: " << BKG.size() << std::endl;
	}

	return;

}

void LocoMouse::loadCalibration() {

	cv::Mat Mtemp;
	cv::FileStorage model_file(CALIBRATION_FILE, cv::FileStorage::READ);

	if (!model_file.isOpened()) {
		//FIXME: Find a way to throw an error here.
		////return -1;
	}

	model_file["ind_warp_mapping"] >> CALIBRATION;
	if (CALIBRATION.empty()) {
		std::cout << "ind_warp_mapping is empty or undefined!" << std::endl;
		//FIXME: Find a way to throw an error here.
		////return -1;
	}

	//model_file["inv_ind_warp_mapping"] >> calibration_inv;

	model_file["view_boxes"] >> Mtemp;
	if (Mtemp.empty()) {
		std::cout << "view_boxes is empty or undefined!" << std::endl;
		//FIXME: Find a way to throw an error here.
		////return -1;
	}
	model_file.release();

	if ((Mtemp.rows != 2) || (Mtemp.cols != 4)) {
		std::cout << "view_boxes sould be a 2x4 matrix!" << std::endl;
		//FIXME: Find a way to throw an error here.
		////return -1;
	}

	if (Mtemp.type() != CV_32SC1) {
		std::cout << "Bounding boxes must be defined with integer pixel positions!" << std::endl;
		//FIXME: Find a way to throw an error here.
		////return -1;
	};

	int *pM = Mtemp.ptr<int>(0);

	BB_SIDE_VIEW.x = pM[0];
	BB_SIDE_VIEW.y = pM[1];
	BB_SIDE_VIEW.width = pM[2];
	BB_SIDE_VIEW.height = pM[3];

	BB_BOTTOM_VIEW.x = pM[4];
	BB_BOTTOM_VIEW.y = pM[5];
	BB_BOTTOM_VIEW.width = pM[6];
	BB_BOTTOM_VIEW.height = pM[7];

	return;

}

void LocoMouse::loadFlip() {
	//FIXME: Find a way to throw exceptions here.
	if (FLIP_CHAR.length() != 1) {
		std::cout << "Mouse side option must be either \"L\" or \"R\"." << std::endl;
		////return -1;
	}

	if (FLIP_CHAR[0] == 'L') {
		IMAGE_FLIP = true;
		//return 1;
	}
	else if (FLIP_CHAR[0] == 'R') {
		IMAGE_FLIP = false;
		//return 1;
	}
	else {
		std::cout << "Mouse side option must be either \"L\" or \"R\"." << std::endl;
		////return -1;
	}

	return;
};

void LocoMouse::validateImageVideoSize() {
	//Checking if the video size and the calibration match:
	//Corrected image size:
	N_ROWS = CALIBRATION.rows;
	N_COLS = CALIBRATION.cols;
	double min_calib_val, max_calib_val;
	cv::minMaxLoc(CALIBRATION, &min_calib_val, &max_calib_val);

	const uint N_row_video = V.get(CV_CAP_PROP_FRAME_HEIGHT);
	const uint N_col_video = V.get(CV_CAP_PROP_FRAME_WIDTH);

	//Validating background images:
	if ((BKG.cols != N_col_video) | (BKG.rows != N_row_video)) {
		throw std::runtime_error(Stream_Formatter() << "Error: Background image does not match video size. Background image has size " << BKG.size() << " while Video has size [" << N_col_video << " x " << N_row_video << "]." << '\n');
	}

	if (LM_PARAMS.LM_DEBUG) {
		DEBUG_TEXT << "Video size: [" << N_col_video << " x " << N_row_video << "]" << std::endl;
	}

	if (LM_PARAMS.LM_DEBUG) {
		DEBUG_TEXT << "[min max] calibration mapping indices: [" << min_calib_val << " x " << max_calib_val << "]" << std::endl;
		DEBUG_TEXT << min_calib_val << std::endl;
		DEBUG_TEXT << max_calib_val << std::endl;
	}

	if ((int)min_calib_val < 0 | (int)max_calib_val >= (N_row_video*N_col_video)) {
		std::cout << "Error: Calibration mapping indices out of range. Minimum index is 0, found  " << min_calib_val << "; Maximum index is : " << (N_row_video*N_col_video) << ", found " << max_calib_val << std::endl;
		//FIXME: Call exception!
		////return -1;
	}
}

void LocoMouse::configurePath() {

#ifdef _WIN32
	//Needed for path manipulation on windows 
	char path_buffer[_MAX_PATH];
	char drive[_MAX_DRIVE];
	char dir[_MAX_DIR];
	char fname[_MAX_FNAME];
	char ext[_MAX_EXT];
	char drive_out[_MAX_DRIVE];
	char dir_out[_MAX_DIR];

	_splitpath_s(LM_CALL.c_str(), drive, dir, fname, ext);
	_makepath_s(path_buffer, drive, dir, "", "");
	ref_path = std::string(path_buffer);

	OUTPUT_PATH += "\\";

	_splitpath_s(VIDEO_FILE.c_str(), drive_out, dir_out, fname, ext);
	output_file = OUTPUT_PATH + "output_" + std::string(fname) + ".yml";
	debug_file = OUTPUT_PATH + "debug_" + std::string(fname) + ".yml";
	debug_text = OUTPUT_PATH + "debug_" + std::string(fname) + ".txt";

#elif __linux__
	OUTPUT_PATH += "/";
	ref_path = std::string(dirname(LM_CALL.c_str()));
	output_file = OUTPUT_PATH + "/output_" + output_file_name(video) + "yml";
	debug_file = OUTPUT_PATH + "/debug_" + output_file_name(video) + "yml";

#endif

}

void LocoMouse::computeBoundingBox() {
	//FIXME: The median filter is an option for this method in particular. Define it properly in the config.yml file as such.

	if (LM_PARAMS.LM_DEBUG) {

		DEBUG_TEXT << "===== Computing the bounding box coordinates: " << std::endl;

	}


	uint padding_offset = LM_PARAMS.median_filter_size / 2;
	uint padding_size = padding_offset * 2;
	
	cv::Mat I_median = cv::Mat::zeros(N_ROWS + padding_size, N_COLS + padding_size, CV_8UC1); //Padded image for median filtering.
	cv::Rect real_I = cv::Rect(padding_offset, padding_offset, N_COLS, N_ROWS); //ROI of the real image i.e. offsetting the padding.

	if (LM_PARAMS.LM_DEBUG) {

		DEBUG_TEXT << "padding_offset: " << padding_offset << std::endl;
		DEBUG_TEXT << "padding_size: " << padding_size << std::endl;
		DEBUG_TEXT << "I_median.size(): " << I_median.size() << std::endl;
		DEBUG_TEXT << "Rect real_I [center part of I_median]: " << real_I << std::endl;
	}


	//Binding I to the central part of I_median:
	cv::Mat I_center = I_median(real_I);

	cv::Mat I_bottom_view = I_center(BB_BOTTOM_VIEW);
	cv::Mat I_side_view = I_center(BB_SIDE_VIEW);

	if (LM_PARAMS.LM_DEBUG) {

		DEBUG_TEXT << "I_center.size(): " << I_center.size() << std::endl;
		DEBUG_TEXT << "I_bottom_view.size(): " << I_bottom_view.size() << std::endl;
		DEBUG_TEXT << "I_side_view.size(): " << I_side_view.size() << std::endl;
	}
	

	std::vector<double> bb_x(N_FRAMES), bb_y_bottom(N_FRAMES), bb_y_side(N_FRAMES), bb_width(N_FRAMES), bb_height_bottom(N_FRAMES), bb_height_side(N_FRAMES);

	if (LM_PARAMS.LM_DEBUG) {
		DEBUG_TEXT << "----- Computing the bounding box position per frame: " << I_center.size() << std::endl;
	}

	for (unsigned int i_frames = 0; i_frames < N_FRAMES; ++i_frames) {

		readFrame(I_center);
		if (LM_PARAMS.LM_DEBUG) {
			DEBUG_TEXT << "Read frame " << i_frames << std::endl;
		}
		computeMouseBox(I_median, I_center, I_side_view, I_bottom_view, bb_x[i_frames], bb_y_bottom[i_frames], bb_y_side[i_frames], bb_width[i_frames], bb_height_bottom[i_frames], bb_height_side[i_frames], LM_PARAMS);
		
		//Bounding box coordinates should be absolute (i.e. regarding corrected image I)
		bb_y_bottom[i_frames] += BB_BOTTOM_VIEW.y;

	}

	//Post processing per-frame boxes to compute final bounding box sizes:
	//Computing size of BB:
	computeMouseBoxSize(bb_width, bb_height_bottom, bb_height_side, BB_SIDE_MOUSE, BB_BOTTOM_MOUSE);

	if (LM_PARAMS.LM_DEBUG) {
		DEBUG_TEXT << "----- Final BB sizes: " << std::endl;
		DEBUG_TEXT << "BB_SIDE_MOUSE: " << BB_SIDE_MOUSE << std::endl;
		DEBUG_TEXT << "BB_BOTTOM_MOUSE: " << BB_BOTTOM_MOUSE << std::endl;
	}

	//Smoothing BB trajectory:
	vecmovingaverage(bb_x, BB_X_POS, LM_PARAMS.moving_average_window);
	vecmovingaverage(bb_y_bottom, BB_Y_BOTTOM_POS, LM_PARAMS.moving_average_window);
	vecmovingaverage(bb_y_side, BB_Y_SIDE_POS, LM_PARAMS.moving_average_window);

	if (LM_PARAMS.LM_DEBUG) {
		DEBUG_TEXT << "===== END " << std::endl << std::endl;
	}

}

void LocoMouse::initializeFeatureLoop() {
	/* Initializes quantities that are dependent on the bounding box size.

	   FIXME: Encapsulate each of these parts into a sub-function.

	*/

	//Initializing I according to the newly computed bounding boxes and pad sizes:
	std::vector <int> zero_pad_rows_pre_vec = { BB_SIDE_MOUSE.height , M.size_pre_side().height , M.size_pre_bottom().height };
	std::vector <int> zero_pad_cols_pre_vec = { BB_BOTTOM_MOUSE.width, M.size_pre_side().width, M.size_pre_bottom().width };

	PAD_PRE_ROWS = *std::max_element(zero_pad_rows_pre_vec.begin(), zero_pad_rows_pre_vec.end());
	PAD_POST_ROWS = (M.size_post_bottom().height > M.size_post_side().height) ? M.size_post_bottom().height : M.size_post_side().height;

	PAD_PRE_COLS = *std::max_element(zero_pad_cols_pre_vec.begin(), zero_pad_cols_pre_vec.end());
	PAD_POST_COLS = (M.size_post_bottom().width > M.size_post_side().width) ? M.size_post_bottom().width : M.size_post_side().width;

	I_PAD = cv::Mat::zeros(PAD_PRE_ROWS + N_ROWS + PAD_POST_ROWS, PAD_PRE_COLS + N_COLS + PAD_POST_COLS, CV_8UC1);
	I_PREV_PAD = cv::Mat::zeros(I_PAD.size(), CV_8UC1);

	I_UNPAD = cv::Rect(PAD_PRE_COLS, PAD_PRE_ROWS, N_COLS, N_ROWS);

	I = I_PAD(I_UNPAD);

	//Initializing I_BB...
	BB_BOTTOM_MOUSE_PAD = cv::Rect(0, 0, M.size_pre_bottom().width + BB_BOTTOM_MOUSE.width + M.size_post_bottom().width, M.size_pre_bottom().height + BB_BOTTOM_MOUSE.height + M.size_post_bottom().height);
	BB_SIDE_MOUSE_PAD = cv::Rect(0, 0, M.size_pre_side().width + BB_SIDE_MOUSE.width + M.size_post_side().width, M.size_pre_side().height + BB_SIDE_MOUSE.height + M.size_post_side().height);

	//Unpad boxes:
	BB_UNPAD_MOUSE_BOTTOM = cv::Rect(M.size_pre_bottom().width, M.size_pre_bottom().height, BB_BOTTOM_MOUSE.width, BB_BOTTOM_MOUSE.height);
	BB_UNPAD_MOUSE_SIDE = cv::Rect(M.size_pre_side().width, M.size_pre_side().height, BB_SIDE_MOUSE.width, BB_SIDE_MOUSE.height);

	//Initializing Tail Boxes:
	TRACKS_TAIL.reserve(N_FRAMES);

	unsigned int tail_box_width = ((int)(double)(BB_BOTTOM_MOUSE.width)*LM_PARAMS.tail_sub_bounding_box);

	BB_BOTTOM_TAIL_PAD = cv::Rect(0, 0, tail_box_width + M.size_pre_bottom().width + M.size_post_bottom().width, BB_BOTTOM_MOUSE.height + M.size_pre_bottom().height + M.size_post_bottom().height);
	BB_UNPAD_TAIL_BOTTOM = cv::Rect(M.size_pre_bottom().width, M.size_pre_bottom().height, tail_box_width, BB_BOTTOM_MOUSE.height);

	BB_BOTTOM_TAIL = cv::Rect(0, 0, tail_box_width, BB_BOTTOM_MOUSE.height);

	BB_SIDE_TAIL_PAD = cv::Rect(0, 0, tail_box_width + M.size_pre_side().width + M.size_post_side().width, BB_SIDE_MOUSE.height + M.size_pre_side().height + M.size_post_side().height);
	BB_UNPAD_TAIL_SIDE = cv::Rect(M.size_pre_side().width, M.size_pre_side().height, tail_box_width, BB_SIDE_MOUSE.height);

	BB_SIDE_TAIL = cv::Rect(0, 0, tail_box_width, BB_BOTTOM_MOUSE.height);

	//Initializing the ONG and dependent variables:
	//FIXME: Center the grid on the image using integer division:
	//Grid of Occlusion Points: 
	unsigned int ngrid_y = ((BB_BOTTOM_MOUSE.height - LM_PARAMS.occlusion_grid_spacing_pixels) / LM_PARAMS.occlusion_grid_spacing_pixels) + 1;
	unsigned int ngrid_x = ((LM_PARAMS.body_proportion_bounding_box * BB_BOTTOM_MOUSE.width) - LM_PARAMS.occlusion_grid_spacing_pixels) / LM_PARAMS.occlusion_grid_spacing_pixels + 1;

	unsigned int Nong = ngrid_x * ngrid_y;

	ONG_size = cv::Size(ngrid_x, ngrid_y);

	ONG_BR_corner = Point_<double>(BB_BOTTOM_MOUSE.width - 1 - LM_PARAMS.occlusion_grid_spacing_pixels / 2, BB_BOTTOM_MOUSE.height - 1 - LM_PARAMS.occlusion_grid_spacing_pixels / 2);

	ONG.reserve(Nong);
	Point_<double> P = (0, 0);
	for (unsigned int i = 0; i < Nong; ++i) {
		ONG.push_back(P);
	}

	for (unsigned int j = 0; j < ngrid_y; ++j) {

		for (unsigned int i = 0; i < ngrid_x; ++i) {

			ONG[j * ngrid_x + i] = ONG_BR_corner - Point_<double>(i * LM_PARAMS.occlusion_grid_spacing_pixels, j * LM_PARAMS.occlusion_grid_spacing_pixels);

		}
	}

	//Side view ONG:
	unsigned int Nong_top = ((BB_SIDE_MOUSE.height - LM_PARAMS.occlusion_grid_spacing_pixels) / LM_PARAMS.occlusion_grid_spacing_pixels) + 1;
	ONG_SIDE.reserve(Nong_top);

	ONG_SIDE_LOWEST_POINT = (BB_SIDE_MOUSE.height - 1 - LM_PARAMS.occlusion_grid_spacing_pixels / 2);

	for (unsigned int i_top = 0; i_top < Nong_top; i_top++) {
		ONG_SIDE.push_back(ONG_SIDE_LOWEST_POINT - (i_top * LM_PARAMS.occlusion_grid_spacing_pixels_top));
	}

	//Resetting the video pointer as the computations of the bounding box could have read it:
	V.set(CV_CAP_PROP_POS_FRAMES, 0);
	CURRENT_FRAME = 0;

}

void LocoMouse::detectBottomCandidates() {
	//FIXME: The mask sets a lot of points to 0. But many of those are already negative, so its a waste of time. Any way to speed this up?

	//Masking the mouse:
	cv::Mat I_bb_bottom_mask;
	threshold(I_BOTTOM_MOUSE, I_bb_bottom_mask, 25.5, 255, cv::THRESH_BINARY_INV); //Threshold at 0.1 on [0 1] range.
	I_bb_bottom_mask(BB_BOTTOM_TAIL).setTo(255, TAIL_MASK); //Setting tail area to 0.

	//Detecting Paw Candidates:
	CANDIDATES_BOTTOM_PAW.push_back(detectPointCandidatesBottom(I_BOTTOM_MOUSE_PAD, BB_UNPAD_MOUSE_BOTTOM, M.paw, I_bb_bottom_mask));

	//Detecting Snout Candidates:
	CANDIDATES_BOTTOM_SNOUT.push_back(detectPointCandidatesBottom(I_BOTTOM_MOUSE_PAD, BB_UNPAD_MOUSE_BOTTOM, M.snout, I_bb_bottom_mask));
}

void LocoMouse::detectSideCandidates() {
	//Masking the mouse:
	cv::Mat I_bb_side_mask;
	threshold(I_SIDE_MOUSE, I_bb_side_mask, 25.5, 255, cv::THRESH_BINARY_INV); //Threshold at 0.1 on [0 1] range.

	//Detecting Paw Candidates:
	if (CANDIDATES_BOTTOM_PAW.back().size() > 0)
		CANDIDATES_SIDE_PAW.push_back(detectPointCandidatesSide(I_SIDE_MOUSE_PAD, BB_UNPAD_MOUSE_SIDE, M.paw, I_bb_side_mask));

	//Detecting Snout Candidates:
	if (CANDIDATES_BOTTOM_SNOUT.back().size() > 0)
		CANDIDATES_SIDE_SNOUT.push_back(detectPointCandidatesSide(I_SIDE_MOUSE_PAD, BB_UNPAD_MOUSE_SIDE, M.snout, I_bb_side_mask));
}


vector <Candidate> LocoMouse::detectPointCandidatesBottom(cv::Mat& I_VIEW_PAD, cv::Rect BB_UNPAD, LocoMouse_Feature M_FEAT, cv::Mat &I_bb_bottom_mask) {

	//Filter the image with the model:
	cv::Mat Filter_Scores_PAD, Filter_Scores;
	cv::filter2D(I_VIEW_PAD, Filter_Scores_PAD, CV_32F, M_FEAT.detector_bottom(), cv::Point(-1, -1), -M_FEAT.rho_bottom(), cv::BORDER_CONSTANT);

	Filter_Scores = Filter_Scores_PAD(BB_UNPAD);

	Filter_Scores.setTo(0, I_bb_bottom_mask);

	//Performing NMS
	return nmsMax(Filter_Scores, M_FEAT.size_bottom(), 0.5); //T = 0.5;

}

vector <Candidate> LocoMouse::detectPointCandidatesSide(cv::Mat& I_VIEW_PAD, cv::Rect BB_UNPAD, LocoMouse_Feature M_FEAT, cv::Mat &I_bb_bottom_mask) {

	//Filter the image with the model:
	cv::Mat Filter_Scores_PAD, Filter_Scores;
	cv::filter2D(I_VIEW_PAD, Filter_Scores_PAD, CV_32F, M_FEAT.detector_side(), cv::Point(-1, -1), -M_FEAT.rho_side(), cv::BORDER_CONSTANT);

	Filter_Scores = Filter_Scores_PAD(BB_UNPAD);

	Filter_Scores.setTo(0, I_bb_bottom_mask);

	//Performing peakClustering:
	//FIXME: Fix the debug flag
	return peakClustering(Filter_Scores, M_FEAT.size_side(), 3, 0.5, false); //max, T = 0.5;

}


void LocoMouse::computeUnaryCostsBottom() {

	//Compute Unary potentials:
	UNARY_BOTTOM_PAW.push_back(unaryCostBox(CANDIDATES_BOTTOM_PAW.back(), BB_BOTTOM_MOUSE, LM_PARAMS.PRIOR_PAW));

	UNARY_BOTTOM_SNOUT.push_back(unaryCostBox(CANDIDATES_BOTTOM_SNOUT.back(), BB_BOTTOM_MOUSE, LM_PARAMS.PRIOR_SNOUT));

	return;
}

void LocoMouse::computePairwiseCostsBottom() {

	//Computing pairwise potentials:
	MATSPARSE temp = pairwisePotential(CANDIDATES_BOTTOM_PAW.end()[-2], CANDIDATES_BOTTOM_PAW.end()[-1], ONG_BR_corner, (double)LM_PARAMS.occlusion_grid_spacing_pixels, ONG, ONG_size, LM_PARAMS.max_displacement, LM_PARAMS.alpha_vel, LM_PARAMS.pairwise_occluded_cost);
	PAIRWISE_BOTTOM_PAW.push_back(temp);

	temp = pairwisePotential(CANDIDATES_BOTTOM_SNOUT.end()[-2], CANDIDATES_BOTTOM_SNOUT.end()[-1], ONG_BR_corner, (double)LM_PARAMS.occlusion_grid_spacing_pixels, ONG, ONG_size, LM_PARAMS.max_displacement, LM_PARAMS.alpha_vel, LM_PARAMS.pairwise_occluded_cost);
	PAIRWISE_BOTTOM_SNOUT.push_back(temp);

}

void LocoMouse::computeMouseBox(cv::Mat &I_median, cv::Mat &I, cv::Mat &I_side_view, cv::Mat &I_bottom_view, double& bb_x, double& bb_y_bottom, double& bb_y_side, double& bb_width, double& bb_height_bottom, double& bb_height_side, const LocoMouse_Parameters &LM_PARAMS) {
	//Compute the bounding box of the mouse for the overground.

	//Median filter to remove noise:
	medianBlur(I_median, I_median, LM_PARAMS.median_filter_size);

	//Thresholding
	cv::threshold(I, I, 2.55, 1, 0); //1% of 255;

	//FIXME: Mouse limits using Connected Components. Move all of this into a function for clarity. 
	cv::Mat labels, stats, centroids;

	uint N_labels = connectedComponentsWithStats(I_side_view, labels, stats, centroids, LM_PARAMS.conn_comp_connectivity, CV_32S);

	//OpenCV assumes the first object to be the background.t
	uint largest_object = 1;
	uint largest_object_area = stats.at<uint>(1, cv::CC_STAT_AREA);
	for (uint i_labels = 2; i_labels < N_labels; ++i_labels) {
		if (stats.at<uint>(i_labels, cv::CC_STAT_AREA) > largest_object_area) {
			largest_object = i_labels;
			largest_object_area = stats.at<uint>(i_labels, cv::CC_STAT_AREA);
		}
	}
	//Selecting largest object:
	compare(labels, largest_object, I_side_view, cv::CMP_EQ);

	//Repeating for bottom view:
	N_labels = connectedComponentsWithStats(I_bottom_view, labels, stats, centroids, LM_PARAMS.conn_comp_connectivity, CV_32S);

	largest_object = 1;
	largest_object_area = stats.at<uint>(1, cv::CC_STAT_AREA);
	for (uint i_labels = 2; i_labels < N_labels; ++i_labels) {
		if (stats.at<uint>(i_labels, cv::CC_STAT_AREA) > largest_object_area) {
			largest_object = i_labels;
			largest_object_area = stats.at<uint>(i_labels, cv::CC_STAT_AREA);
		}
	}

	//Selecting largest object:
	compare(labels, largest_object, I_bottom_view, cv::CMP_EQ);

	//Checking for first and last positions to exceed min_pixel_visible:
	cv::Mat Row_top, Row_bottom, Col_top, Col_bottom;

	cv::reduce(I_side_view, Row_top, 0, CV_REDUCE_SUM, CV_32SC1);
	cv::reduce(I_side_view, Col_top, 1, CV_REDUCE_SUM, CV_32SC1);

	cv::reduce(I_bottom_view, Row_bottom, 0, CV_REDUCE_SUM, CV_32SC1);
	cv::reduce(I_bottom_view, Col_bottom, 1, CV_REDUCE_SUM, CV_32SC1);

	uint32_t lims_row_top[2], lims_col_top[2], lims_row_bottom[2], lims_col_bottom[2];

	//Compute and assign values to bounding box:
	//FIXME: Handle the empty image case. Perform the check inside the function and return a bool. Check after each run if the result was valid.
	//Row_* marks x and with; Col_* marks y and height.
	firstLastOverT(Row_top, I.cols, lims_row_top, LM_PARAMS.min_pixel_visible);
	firstLastOverT(Row_bottom, I.cols, lims_row_bottom, LM_PARAMS.min_pixel_visible);
	firstLastOverT(Col_top, I_side_view.rows, lims_col_top, LM_PARAMS.min_pixel_visible);
	firstLastOverT(Col_bottom, I_bottom_view.rows, lims_col_bottom, LM_PARAMS.min_pixel_visible);

	//Assigning the values to the bounding box vectors:
	bb_x = (lims_row_bottom[1] > lims_row_top[1]) ? (double)lims_row_bottom[1] : (double)lims_row_top[1];
	bb_y_bottom = (double)lims_col_bottom[1];
	bb_y_side = (double)lims_col_top[1];

	//NOTE: By the definition of lims, the real dimensions should be [1]-[0] + 1. But to agree with the way MATLAB crops images, we drop the +1. This creates some inconsistencies as if the whole image is white, the width/height can never be the with/height of the image  
	uint32_t wt = lims_row_top[1] - lims_row_top[0];
	uint32_t wb = lims_row_bottom[1] - lims_row_bottom[0];

	bb_width = (wt > wb) ? (double)wt : (double)wb;
	bb_height_bottom = (double)(lims_col_bottom[1] - lims_col_bottom[0]);
	bb_height_side = (double)(lims_col_top[1] - lims_col_top[0]);

	return;

}

void LocoMouse::matchBottomSideCandidates() {

	//FIXME: Fix the debugging variable
	Point_<int> padding_pre_bottom = Point_<int>(M.size_pre_bottom().width, M.size_pre_bottom().height);
	Point_<int> padding_pre_side = Point_<int>(M.size_pre_side().width, M.size_pre_side().height);

	vector <P22D> P = matchingWithVelocityConstraint(CANDIDATES_BOTTOM_PAW.back(), CANDIDATES_SIDE_PAW.back(), I_BOTTOM_MOUSE_PAD, I_SIDE_MOUSE_PAD, I_BOTTOM_MOUSE_PAD_PREV, I_SIDE_MOUSE_PAD_PREV, padding_pre_bottom, padding_pre_side, CURRENT_FRAME > 0, M.paw, LM_PARAMS.top_bottom_min_overlap, CURRENT_FRAME == 161);
	CANDIDATES_MATCHED_VIEWS_PAW.push_back(P);

	P = matchingWithVelocityConstraint(CANDIDATES_BOTTOM_SNOUT.back(), CANDIDATES_SIDE_SNOUT.back(), I_BOTTOM_MOUSE_PAD, I_SIDE_MOUSE_PAD, I_BOTTOM_MOUSE_PAD_PREV, I_SIDE_MOUSE_PAD_PREV, padding_pre_bottom, padding_pre_side, CURRENT_FRAME > 0, M.snout, LM_PARAMS.top_bottom_min_overlap, false);
	CANDIDATES_MATCHED_VIEWS_SNOUT.push_back(P);

	return;

};

vector<P22D> LocoMouse::matchingWithVelocityConstraint(vector<Candidate>& Candidates_b, vector<Candidate>& Candidates_t, const Mat& Ibbb, const Mat& Itbb, const Mat& Ibbb_prev, const Mat& Itbbb_prev, const Point_<int> padding_pre_bottom, const Point_<int> padding_pre_side, bool vel_check, LocoMouse_Feature& F, double T, bool debug) {
	// BOXPAIRINGS Pairs boxes that overlap at least T horizontally.
	//
	// INPUT :
	// Candidates_b : Candidate vector of bottom positions.
	// Candidate_t: Candidate vector of top positions.
	// moving_b: Boolean checking moving/not_moving status for bottom candidates.
	// movint_t: Boolean checking moving/not_moving status for top candidates.
	// box_width : with of the boxes in pixels.
	// mode : decides the output x coordinate: bottom_candidate(0), max_scores (1), weighted average (2);
	// T : Minimum overlapping ratio to consider pairing (between 0 and 1).
	//
	// Note : The score will always be the sum of both detections, as that is
	// what is going to be used during the multi - object tracking.If that
	// changes, this must be changed accordingly.

	//The original function takes more options for the final 2D coordinate. Only implementing the one actually used which is to keep the bottom one (mode == 0).

	//FIXME: We cannot pass the whole model anymore as it would make the function feature specific...

	int ovlp = (F.size_bottom().width * (1 - T));
	int N_bottom = Candidates_b.size();
	int N_top = Candidates_t.size();
	vector<P22D> Candidates_bt;
	if (N_bottom == 0) {
		return Candidates_bt;
	}

	//Calculate Pairwise Distance Matrix:
	//It would be faster to calculate distances only if motion constraints are valid. But motion constraints are not reliable,thus only if multiple matches exist should they be applied.
	Mat Distances, D_top_weight, boolD;
	if (N_top > 0) {
		Distances = xDist(Candidates_b, Candidates_t);

		boolD = Distances <= ovlp;
		normalize(boolD, boolD, 0, 1, NORM_MINMAX, -1);

		Distances.convertTo(D_top_weight, CV_64FC1);

		D_top_weight = 1 - (D_top_weight / (double)ovlp);
	}

	return matchViews(boolD, D_top_weight, Candidates_b, Candidates_t, vel_check, F, Ibbb, Itbb, Ibbb_prev, Itbbb_prev, padding_pre_bottom, padding_pre_side, debug);
}

cv::Mat LocoMouse::xDist(const vector<Candidate> &P1, const vector <Candidate> &P2) {
	//Computes the distance in pixels on the x axis.	

	cv::Mat D(P1.size(), P2.size(), CV_32SC1);

	if (D.isContinuous()) {
		int* p = D.ptr<int>(0);

		int offset = 0;

		for (int i = 0; i < P1.size(); ++i) {
			for (int j = 0; j < P2.size(); ++j) {

				p[offset + j] = abs(P1[i].point().x - P2[j].point().x);
			}

			offset = offset + P2.size();
		}
	}
	else {
		for (int i = 0; i < P1.size(); ++i) {

			int* p = D.ptr<int>(i);

			for (int j = 0; j < P2.size(); ++j) {

				p[j] = abs(P1[i].point().x - P2[j].point().x);
			}
		}
	}

	return D;
}

vector<P22D> LocoMouse::matchViews(const Mat &boolD, const Mat &D_top_weight, const vector<Candidate> &C_b, const vector<Candidate> &C_t, bool vel_check, LocoMouse_Feature F, const Mat &Ibbb, const Mat &Itbb, const Mat &Ibbb_prev, const Mat &Itbb_prev, const Point_<int> padding_pre_bottom, const Point_<int> padding_pre_side, bool debug) {
	//VERY IMPORTANT: I'm assuming there are not more than 255 candidates per match. This is a very safe bet, as if it happens the data is wrong for sure.
	//FIXME: Do not pass M as argument as the function becomes feature specific
	//FIXME: C_b and C_t are not passed as constant because I don't know how to access the methods of a class when the class is instanciated as constant.
	vector<P22D> C;

	int N_top = C_t.size();
	int N_bottom = C_b.size();

	if (N_bottom == 0) {
		return C;
	}

	//Predefined function constants:
	double moving_alpha_bottom = 0.02;
	double moving_alpha_top = 0.05;
	double moving_threshold = 25;

	//Avoiding the moving status twice for top candidates since we are doing nested loops:
	vector<bool> need_to_check(N_top, true);
	vector<bool> moving_t(N_top, false);

	Mat M_check_velocity, M_t_per_b_match;
	float *top_per_bottom_match = 0, *bottom_per_top_match = 0;

	if (N_top > 0) {
		//Reduction of D to find how many bottom candidates correspond to each top candidate.

		reduce(boolD, M_check_velocity, 0, CV_REDUCE_SUM, CV_32FC1);
		bottom_per_top_match = M_check_velocity.ptr<float>(0);

		//Reduction of D to find how many top candidates correspond to each bottom candidate.

		reduce(boolD, M_t_per_b_match, 1, CV_REDUCE_SUM, CV_32FC1);
		top_per_bottom_match = M_t_per_b_match.ptr<float>(0);

	}

	for (int i_bottom = 0; i_bottom < N_bottom; i_bottom++) {

		//No top candidates at all:
		if (N_top == 0) {
			C.push_back(P22D(C_b[i_bottom], Candidate(-1, -1, -1)));
			continue;
		}

		if (top_per_bottom_match[i_bottom] == 0) {
			//No match at the top:
			//Place the bottom candidate with no top correspondence
			C.push_back(P22D(C_b[i_bottom], Candidate(-1, -1, -1)));
		}

		else {
			//Multiple matches at the top:
			//Checking if we create a P22D candidate or add top coordinates to it.

			bool is_first = true;
			bool moving_b = false;
			bool need_to_check_bottom = true;
			bool match = true;

			//Top view matches to current view:
			const uchar* pD = boolD.ptr<uchar>(i_bottom);
			const double* pDTW = D_top_weight.ptr<double>(i_bottom);

			for (int i_top = 0; i_top < N_top; i_top++) {

				if (pD[i_top] < 1)
					continue;

				//Check the need for velocity constraint checking for current top view candidate
				if (bottom_per_top_match[i_top] > 1 & vel_check) {

					//If it's the first time processing this bottom candidate, check its moving status:
					if (need_to_check_bottom) {
						moving_b = checkVelCriterion(Ibbb, Ibbb_prev, F.match_box_bottom() + C_b[i_bottom].point() + padding_pre_bottom, F.size_bottom().area(), moving_alpha_bottom, moving_threshold);

						need_to_check_bottom = false;
					}

					//If it's the first time processing this top candidate, check its moving status:
					if (need_to_check[i_top]) {

						moving_t[i_top] = checkVelCriterion(Itbb, Itbb_prev, F.match_box_side() + C_t[i_top].point() + padding_pre_side, F.size_side().area(), moving_alpha_top, moving_threshold);

						need_to_check[i_top] = false;
					}

					match = moving_b == moving_t[i_top];
				}
				else {
					match = true;
				}

				if (match) {

					Candidate C_temp = Candidate(C_t[i_top].point(), C_t[i_top].score() * pDTW[i_top]);

					if (is_first) {
						C.push_back(P22D(C_b[i_bottom], C_temp));
						is_first = false;
					}
					else {

						C[C.size() - 1].add_top_candidate(C_temp);
					}
				}

			}

			if (is_first) {
				//Place the bottom candidate with no top correspondence
				C.push_back(P22D(C_b[i_bottom], Candidate(-1, -1, -1)));
			}
		}
	}

	return C;
}

bool LocoMouse::checkVelCriterion(const Mat& I, const Mat& I_prev, const Rect& im_box, int box_area, double alpha, double T) {
	//Mat Ii, Iim1, S;
	//Rect bb_aux(0, 0, im_box.width, im_box.height);

	////FIXME: Replace this by cropping on a padded image!
	//imBorderPreservingBB(I, Ii, bb_aux, im_box.x, im_box.y);
	//imBorderPreservingBB(I_prev, Iim1, bb_aux, im_box.x, im_box.y);

	cv::Mat S;
	cv::subtract(I(im_box), I_prev(im_box), S, noArray(), CV_8UC1);

	cv::threshold(S, S, T, 1, THRESH_BINARY);

	cv::Scalar sum_S = sum(S);

	//FIXME: box_area*alpha can be precomputed for every detector size!
	return (double)sum_S(0) >= ((double)box_area) * alpha;
}


void firstLastOverT(const cv::Mat& values, const uint L, uint32_t first_last[2], const uint T) {
	//Goes over all points in the vector checking if they are above the threshold. The first one triggers a flag so that the next one(s) are saved as the final point. The last one is kept.
	//FIXME: What if there are no values?
	bool has_first = false;

	const float* p = values.ptr<float>(0);

	first_last[0] = 0;
	first_last[1] = L - 1;

	for (uint i = 0; i < L; ++i) {
		if (p[i] > T) {

			if (has_first) {
				first_last[1] = (uint32_t)i;
			}
			else
			{
				first_last[0] = (uint32_t)i;
				has_first = true;
			}

		}
	}
	return;
}

void LocoMouse::readFrame() {
	LocoMouse::readFrame(I);
}

void LocoMouse::readFrame(cv::Mat &I) {
	//FIXME: What is a good way to implement a debug flag?
	bool debug = false;

	//Reading next frame:
	if (debug)
		DEBUG_TEXT << "readLocoMouse" << std::endl;

	cv::Mat F;
	V >> F;
	if (!F.data) {
		std::cout << "Error reading image from video!" << std::endl;
		//FIXME: Throw exception here.
	}

	if (debug)
		DEBUG_TEXT << "read Image ok" << std::endl;

	extractChannel(F, F, 0); //Although video is grayscale opencv always reads with 3 channels.

	//if (F.type() == CV_8UC3) {
	//	//FIXME: Don't convert as this implies computations. Simply get the first channel.
	//	cvtColor(F, F, CV_BGR2GRAY); //The system sometimes outputs color videos, don't know why. Its a waste of time.
	//	if (debug)
	//		DEBUG_TEXT << "convertColorOK" << std::endl;
	//}

	//Removing background:
	//FIXME: Check if cropping the BB beforehand saves computational time.
	subtract(F, BKG, F);

	if (debug)
		DEBUG_TEXT << "subtract OK" << std::endl;

	//Normalize the range to [0 255]
	normalize(F, F, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	if (debug)
		DEBUG_TEXT << "norm OK" << std::endl;

	//Undistorting the image:
	correctImage(F, I);

	if (debug)
		DEBUG_TEXT << "calib OK" << std::endl;

	//FIXME: Should flipping be done just for the BB's? Less pixels, probably faster.
	//Flipping if needed:
	if (IMAGE_FLIP) {
		flip(I, I, 1);
		if (debug)
			DEBUG_TEXT << "flip OK" << std::endl;
	}

	return;

}



void LocoMouse::correctImage(cv::Mat& Iin, cv::Mat& Iout) {
	//FIXME: How to pass the debug flag appropriately?
	bool debug = false;

	//Applies the mapping such that Iout[x] = Iin[Calibration[x]];
	CV_Assert(Iout.size() == CALIBRATION.size());

	int max_dim;
	if (debug) {
		DEBUG_TEXT << Iin.size() << std::endl;
		DEBUG_TEXT << Iout.size() << std::endl;
		DEBUG_TEXT << CALIBRATION.size() << std::endl;
		max_dim = Iin.rows*Iin.cols;
		DEBUG_TEXT << "Maxdim is: " << max_dim << std::endl;
	}

	Iin.reshape(0, 1);

	if (debug)
		DEBUG_TEXT << "Iin is continuous: " << Iin.isContinuous() << std::endl;

	uint N_cols = CALIBRATION.cols;
	uint N_rows = CALIBRATION.rows;

	if (Iout.isContinuous() & CALIBRATION.isContinuous()) {
		if (debug)
			DEBUG_TEXT << "Continuous" << std::endl;

		N_cols = N_cols * N_rows;
		N_rows = 1;
	}
	else {
		if (debug)
			DEBUG_TEXT << "Not continuous" << std::endl;
	}

	if (debug)
		DEBUG_TEXT << Iin.type() << " " << Iout.type() << " " << CALIBRATION.type() << std::endl;

	uchar* p_Iin = Iin.ptr<uchar>(0);

	for (uint i_rows = 0; i_rows < N_rows; ++i_rows) {

		uchar* p_Iout = Iout.ptr<uchar>(i_rows);
		int32_t* p_C = CALIBRATION.ptr<int32_t>(i_rows);

		for (uint i_cols = 0; i_cols < N_cols; ++i_cols) {

			if (debug) {
				if (p_C[i_cols] >= max_dim) {
					DEBUG_TEXT << "Mapping is: " << p_C[i_cols] << " while max dim is " << max_dim << std::endl;
				}
				CV_Assert(p_C[i_cols] < max_dim);
				CV_Assert(p_C[i_cols] >= 0);
			}

			p_Iout[i_cols] = p_Iin[p_C[i_cols]];
		}
	}

	return;

}

void LocoMouse::cropBoundingBox() {
	/*	Crops the bounding box around the mouse. If the real image is smaller than the bounding box, padding is used.

		Locomouse uses the bottom right coordinate to define the bounding box because mice walk from left to right and thus the bottom righ corner is always visible. Thus BB_X_POS etc are BR coordinates.

		These bounding boxes are then used for filtering. As such they are cropped with extra padding depending on the model sizes so that all filtered values are as accurate as possible (due to the border properties of filtering/convolving)

	*/

	//Moving the padded boxes:
	//The position of the BR corner is computed on the real image coordinates. To get the padding box we must add the I_PAD offsets and remove (BB_*_MOUSE.width + pad_pre) which is equivalent to (BB_*_MOUSE_PAD.width - pad_post).

	BB_BOTTOM_MOUSE_PAD.x = (BB_X_POS[CURRENT_FRAME] + PAD_PRE_COLS) - (BB_BOTTOM_MOUSE_PAD.width - M.size_post_bottom().width) + 1;
	BB_BOTTOM_MOUSE_PAD.y = (BB_Y_BOTTOM_POS[CURRENT_FRAME] + PAD_PRE_ROWS) - (BB_BOTTOM_MOUSE_PAD.height - M.size_post_bottom().height) + 1;

	I_BOTTOM_MOUSE_PAD = I_PAD(BB_BOTTOM_MOUSE_PAD);
	I_BOTTOM_MOUSE = I_BOTTOM_MOUSE_PAD(BB_UNPAD_MOUSE_BOTTOM);

	BB_SIDE_MOUSE_PAD.x = (PAD_PRE_COLS + BB_X_POS[CURRENT_FRAME]) - (BB_SIDE_MOUSE_PAD.width - M.size_post_side().width) + 1;
	BB_SIDE_MOUSE_PAD.y = (PAD_PRE_ROWS + BB_Y_SIDE_POS[CURRENT_FRAME]) - (BB_SIDE_MOUSE_PAD.height - M.size_post_side().height) + 1;

	I_SIDE_MOUSE_PAD = I_PAD(BB_SIDE_MOUSE_PAD);
	I_SIDE_MOUSE = I_SIDE_MOUSE_PAD(BB_UNPAD_MOUSE_SIDE);

	//Moving the padded boxes for the previous image:
	I_BOTTOM_MOUSE_PAD_PREV = I_PREV_PAD(BB_BOTTOM_MOUSE_PAD);
	I_SIDE_MOUSE_PAD_PREV = I_PREV_PAD(BB_SIDE_MOUSE_PAD);

	return;
}


void LocoMouse::computeMouseBoxSize(std::vector<double> &bb_w, std::vector<double> &bb_hb, std::vector<double> &bb_ht, cv::Rect &BB_top, cv::Rect &BB_bottom) {
	/* Post-processes the per-frame bounding boxes to compute a sigle size based on the statistics of the observed boxes over the sequence*/

	unsigned int N_frames = bb_w.size(); //Assumes all others have the same size.

	double median_w = medianvec(bb_w, N_frames);
	double median_hb = medianvec(bb_hb, N_frames);
	double median_ht = medianvec(bb_ht, N_frames);

	double std_w = stdvec(bb_w, N_frames);
	double std_hb = stdvec(bb_hb, N_frames);
	double std_ht = stdvec(bb_ht, N_frames);

	uint32_t median_w_3std = (uint32_t)(median_w + 3 * std_w);
	uint32_t median_hb_3std = (uint32_t)(median_hb + 3 * std_hb);
	uint32_t median_ht_3std = (uint32_t)(median_ht + 3 * std_ht);

	//NOTE: bb vectors are now sorted so the maximum element is the last one!
	uint32_t final_w = (median_w_3std < bb_w[N_frames - 1]) ? median_w_3std : bb_w[N_frames - 1];
	uint32_t final_hb = (median_hb_3std < bb_hb[N_frames - 1]) ? median_hb_3std : bb_hb[N_frames - 1];
	uint32_t final_ht = (median_ht_3std < bb_ht[N_frames - 1]) ? median_ht_3std : bb_ht[N_frames - 1];

	BB_top = cv::Rect(0, 0, final_w, final_ht);
	BB_bottom = cv::Rect(0, 0, final_w, final_hb);
	return;
}

double medianvec(std::vector<double> &v, const int N) {
	//Computing median values

	if (N == 1) {
		return v[0];
	}
	sort(v.begin(), v.end());

	int half_frames = N / 2;
	if (N % 2 == 0)
	{
		double median = (v[half_frames - 1] + v[half_frames]) / 2;
		return median;
	}
	else
	{
		return v[half_frames - 1];
	}
}

double stdvec(std::vector<double> &v, int N) {
	/*Computes the Standard deviation of a vector of N samples*/
	//Computing standard deviations:
	if (N == 1) {
		return 0.0;
	}

	std::vector<double> vv(N);
	for (int i_val = 0; i_val < N; ++i_val) {
		vv[i_val] = (double)v[i_val];
	}

	double sum = std::accumulate(vv.begin(), vv.end(), 0.0);
	double mean = sum / N;

	std::vector<double> diff(N);
	transform(vv.begin(), vv.end(), diff.begin(),
		std::bind2nd(std::minus<double>(), mean));
	double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
	double stdev = sqrt(sq_sum / (N - 1));
	return stdev;
}

void vecmovingaverage(std::vector<double> &v, std::vector<unsigned int> &vout, int N_window) {
	/* Computes the central moving average of a vector.

	In this case we are interested in pixel coordinates so the output is in ints. The values are rounded by integer division.

	It is assumed N_window to be odd.

	The Partial overlap cases (when the window does not fully overlap the input) is left untouched instead of any other border handling case.

	*/

	if (N_window >= v.size()) {

		for (unsigned int i_samples = 0; i_samples < v.size(); ++i_samples) {

			vout[i_samples] = (uint32_t)v[i_samples];

		}
		return;
	}

	double current_sum = 0;
	int N_half_window = N_window / 2;

	//Copying the first N_window/2 values:
	for (int i_samples = 0; i_samples < N_half_window; ++i_samples) {
		vout[i_samples] = (uint32_t)v[i_samples];
	}

	//Summing the first window:
	for (int i_samples = 0; i_samples < N_window; ++i_samples) {
		current_sum += v[i_samples];
	}

	//The first average:
	vout[N_half_window] = (uint32_t)floor(current_sum / N_window);//int division!

																  //The valid values:
	for (int i_samples = 0; i_samples < v.size() - N_window; ++i_samples) {
		current_sum = current_sum - v[i_samples] + v[i_samples + N_window];
		vout[N_half_window + 1 + i_samples] = (uint32_t)floor(current_sum / N_window);
	}

	//The last N_window/2 values:
	for (int i_samples = v.size() - N_half_window - 1; i_samples < v.size(); ++i_samples) {
		vout[i_samples] = (uint32_t)v[i_samples];
	}
}

vector<Candidate> nmsMax(const Mat& detection_box, Size box_size, double overlap) {
	/* nmsMax performs non - maxima suppresion in a volume using the Pascal
	criterium of intersection_area / union_area > overlap->suppression.

	Based on MATLAB code by Piotr Dollar (Piotr's MATLAB Toolbox).

	Similar to peakClustering, but more agressive suppression.

	The output coordinate is the weighted average of the points suppressed by each maxima.

	The output score is the score of the maxima.

	*/

	if (!detection_box.data) {
		vector<Candidate> empty;
		return empty;
	}

	//ofstream temp("temp.txt");

	//Extract positive detections and sort:
	vector <Candidate> detections;

	int N_cols = detection_box.cols;
	int N_rows = detection_box.rows;

	//FIXME: What if matrix is continuous?
	for (int i_rows = 0; i_rows < N_rows; ++i_rows) {

		const float* db_p = detection_box.ptr<float>(i_rows);

		for (int i_cols = 0; i_cols < N_cols; ++i_cols) {

			if (db_p[i_cols] > 0) {
				detections.push_back(Candidate(i_cols, i_rows, db_p[i_cols]));
			}
		}
	}

	uint N_detections = detections.size();

	if (N_detections == 0) {
		vector<Candidate> empty;
		return empty;
	}

	//Sort candidates according to score:
	sort(begin(detections), end(detections), compareCandidate);

	//-- Perform Non-maxima suppression keeping track of which maxima supresses each point.

	vector <unsigned int> maxima_index(N_detections);
	vector <unsigned int> candidate_index;
	uint candidate_counter = 0;
	std::map<unsigned int, unsigned int> maxima_index_mapping;

	//Pre-computing box parameters:
	double area2 = 2 * box_size.area();

	//Points to suppress:
	vector<bool> discard(N_detections, false);

	//Going over the point candidates:
	Rect Rect_1(0, 0, box_size.width, box_size.height);
	Rect Rect_2(0, 0, box_size.width, box_size.height);

	for (uint i = 0; i < N_detections; ++i) {
		//Move Rect_1:
		Rect_1 += detections[i].point();

		//If the point is not suppressed, set its maxima index to itself:
		if (!discard[i]) {
			candidate_index.push_back(i);
			maxima_index[i] = i;
			maxima_index_mapping[i] = candidate_counter;
			candidate_counter += 1;
		}

		//Suppressing points:
		for (uint j = i + 1; j < N_detections; ++j) {

			if (discard[j])
				continue;

			//Move Rect_2:
			Rect_2 += detections[j].point();

			//Check overlap:
			Rect R = (Rect_1 & Rect_2);

			if (R.area() == 0) {
				//Resetting Rect_2:
				Rect_2 -= detections[j].point();
				continue;
			}

			double criterion = R.area() / (area2 - R.area());

			if (criterion > overlap) {
				discard[j] = true;
				maxima_index[j] = maxima_index[i];
			}

			//Resetting Rect_2:
			Rect_2 -= detections[j].point();
		}

		//Resetting Rect_1:
		Rect_1 -= detections[i].point();
	}

	//Debugging:
	uint N_candidates = candidate_index.size();

	//Computing the weighted average based on the cluster size:
	vector<Candidate> candidates(N_candidates);
	vector < Point_<double> > weighted_mean(N_candidates);
	vector <double> sumscores(N_candidates);

	//Computing the weighted mean for each maxima cluster:
	for (uint i_detections = 0; i_detections < N_detections; ++i_detections) {
		double x, y;
		x = detections[i_detections].point().x;
		y = detections[i_detections].point().y;

		weighted_mean[maxima_index_mapping[maxima_index[i_detections]]] += Point_<double>(x, y) * detections[i_detections].score();

		sumscores[maxima_index_mapping[maxima_index[i_detections]]] += detections[i_detections].score();
	}

	//Assigning the result to the candidate vector:
	for (uint i_candidates = 0; i_candidates < N_candidates; ++i_candidates) {
		candidates[i_candidates] = Candidate(weighted_mean[i_candidates] / sumscores[i_candidates], detections[candidate_index[i_candidates]].score());
	}

	return candidates;
}

vector<Candidate> peakClustering(const Mat& detection_box, Size box_size, int cluster_method, double overlap, bool debug) {
	/*PEAKCLUSTERING Clusters detections that overlap. The new center is their
	weighted mean by the detection scores.The new score is the median
	detection score.

	INPUT :
	detection_box: Filtered image.
	box_size: size of the box to compare.
	cluster_method: an int defining the cluster score method as the maximum (0), mean (1) , median (2) or sum (3) of the scores of clustered points.
	NOTE: Only maximum is implemented so far.
	overlap: a value between 0 and 1 deciding the overlap percentage of the boxes.
	*/

	vector <Candidate> candidates;
	int candidate_counter = 0;

	if (!detection_box.data) {
		return candidates;
	}

	//Extracting the coordinates of points with positive detection :
	vector <Candidate> detections;
	int N_cols = detection_box.cols;
	int N_rows = detection_box.rows;

	//FIXME: Not checking if the image is continuous as it is from a bounding box. If in the future it is seen that operations on full images are actually faster due to continuous memory, perform the check.

	for (int i_rows = 0; i_rows < N_rows; ++i_rows) {

		const float* db_p = detection_box.ptr<float>(i_rows);

		for (int i_cols = 0; i_cols < N_cols; ++i_cols) {
			if (db_p[i_cols] > 0) {
				Candidate c(i_cols, i_rows, db_p[i_cols]);
				detections.push_back(c);
			}
		}
	}

	int N_points = detections.size();

	/*if (debug) {
	DEBUG_TEXT << "Peak Clustering: " << N_points << " detections." << endl;
	}*/

	if (N_points == 0) {
		return candidates;
	}


	//Sort candidates according to score:
	sort(begin(detections), end(detections), compareCandidate);

	//Pre-computing box parameters:
	double area2 = 2 * box_size.height * box_size.width;

	vector<bool> kp(N_points, false);

	//Checking the center coordinate mode:
	//FIXME: By using Rect it shouldn't matter as the offset is the same to all points. Since no information about the image is being used, there should be no out of bounds issues.

	//Going over the point candidates:
	Rect Rect_1(0, 0, box_size.width, box_size.height);
	Rect Rect_2(0, 0, box_size.width, box_size.height);
	Point_<int> P(0, 0);
	int counter = 0;
	for (int i = 0; i < N_points; ++i) {

		if (kp[i])
			continue;
		//Check with a fresh mind what the current cluster variable really does.
		vector<Candidate> current_cluster; //This is copying values. Is it really the best way?
		current_cluster.push_back(detections[i]);

		//Move Rect_1:
		Rect_1 += detections[i].point();
		for (int j = i + 1; j < N_points; ++j) {

			if (kp[j])
				continue;

			//Move Rect_2:
			Rect_2 += detections[j].point();

			//Check overlap:
			Rect R = (Rect_1 & Rect_2);

			if (R.area() == 0) {
				//Resetting Rect_2:
				Rect_2 -= detections[j].point();
				continue;
			}

			double criterion = R.area() / (area2 - R.area());

			if (criterion > overlap) {
				kp[j] = true; //Skipping.
				current_cluster.push_back(detections[j]);
			}
			//Resetting Rect_2:
			Rect_2 -= detections[j].point();
		}

		//Compute the weighted average of the points of the cluster:
		//Understand how this can be done using accumulator. For now, a bit advanced.
		if (current_cluster.size() > 1) {
			Point_<double> peak_point(0, 0);
			double sum_peak_score = 0;
			vector<double> cluster_scores(current_cluster.size());

			/*if (debug && counter == 2) {
			for (uint i_cluster = 0; i_cluster < current_cluster.size(); i_cluster++) {
			myfile << current_cluster[i_cluster].point() << " " << current_cluster[i_cluster].score() << endl;
			}
			}*/

			for (uint i_cluster = 0; i_cluster < current_cluster.size(); i_cluster++) {
				peak_point += Point_<double>(current_cluster[i_cluster].point().x, current_cluster[i_cluster].point().y)*current_cluster[i_cluster].score();
				sum_peak_score += current_cluster[i_cluster].score();
				cluster_scores[i_cluster] = current_cluster[i_cluster].score();
			}

			/*if (debug){

			cout << "Peak point: " << peak_point << endl;
			cout << "sum_peak_score: " << sum_peak_score << endl;
			cout << "Result: " << peak_point / sum_peak_score << endl;
			cout << "Round: " << round(0.5) << endl;
			cout << "WTF: " << round(peak_point.y / sum_peak_score) << endl;
			}*/

			//FIXME: Check divide by zeros etc.
			peak_point = peak_point / sum_peak_score;
			//double peak_score = sum_peak_score / current_cluster.size(); //FIXME: MATLAB uses max!
			candidates.push_back(Candidate(Point_<int>(round(peak_point.x), round(peak_point.y)), detections[i].score()));

		}
		else {
			candidates.push_back(current_cluster[0]);
		}

		counter += 1;

		//FIXME: Add the other modes if wanted, but the alogrithm only uses max.
		/*switch (cluster_method) {
		case 3:
		//Compute the median of the values
		break;
		}*/
		//Resetting Rect_1:
		Rect_1 -= detections[i].point();
	}
	//myfile.close();
	
	return candidates;

}



MyMat LocoMouse::unaryCostBox(vector<Candidate> &p_candidates, Rect &BB, vector<LocoMouse_LocationPrior> location_prior) {

	bool debug = true; //FIXME: Set debug flag for all functions.

	//Determining outputsize:
	int N_candidates = p_candidates.size();
	int N_features = location_prior.size();

	MyMat M(N_candidates, N_features); //Initialized as zeros!
	double norm_fact = 1 / sqrt(2);

	for (int i = 0; i < N_candidates; ++i) {

		Point_<double> candidate = Point_<double>((double)p_candidates[i].point().x / (double)BB.width, (double)p_candidates[i].point().y / (double)BB.height);

		for (int j = 0; j < N_features; ++j) {
			if (candidate.inside(location_prior[j].area())) {
				if (debug) {
					DEBUG_TEXT << "Location prior " << i << ": " << location_prior[j].position() << endl;
				}

				//Distance to location prior point:
				Point_<double> distance = candidate - location_prior[j].position();
				double val = sqrt(distance.dot(distance)) * norm_fact;

				if (debug) {
					DEBUG_TEXT << "val: " << val << " max_distance: " << location_prior[j].max_distance() << endl;
				}

				if (val <= location_prior[j].max_distance()) {
					M.put(i, j, (1 - val) * p_candidates[i].score());
				}
			}
		}
	}
	DEBUG_TEXT.close();
	return M;
}

MATSPARSE LocoMouse::pairwisePotential(vector<Candidate> &Ci, vector<Candidate> &Cip1, Point_<double> &grid_mapping, double grid_spacing, vector< Point_<double> > &ONGi, Size ONG_size, double max_displacement, double alpha_vel, double pairwise_occluded_cost) {

	int Nong = ONGi.size();
	int Ni = Ci.size();
	int Nip1 = Cip1.size();

	int Nong_x = ONG_size.width;
	int Nong_y = ONG_size.height;

	// << "ONG grid size: X-> " << ONG_size.width << " Y-> " << ONG_size.height << endl;

	double pairwise_occluded_cost_alpha_vel = pairwise_occluded_cost * alpha_vel;

	if (isinf(pairwise_occluded_cost_alpha_vel)) {
		cout << "Inf detected" << endl;
	}
	if (isnan(pairwise_occluded_cost_alpha_vel)) {
		cout << "NaN detected" << endl;
	}

	int Nong_x_aux = ONG_size.width - 1;
	int Nong_y_aux = ONG_size.height - 1;

	//DEBUG_TEXT << "Ni: " << Ni << endl;
	//DEBUG_TEXT << "Nip1: " << Nip1 << endl;

	MyMat D = MyMat(Nip1 + Nong, Ni + Nong);

	//DEBUG_TEXT << "Size of transition matrix is: " << D.Nrows() << " " << D.Ncols() << endl;
	Point_<uint32_t> ONG_i;
	Point_<uint32_t> ONG_ip1;

	//DEBUG_TEXT << "BR Coordinates: " << grid_mapping << endl;

	//Fi to Fip1 transitions:
	for (int i = 0; i < Ni; ++i) {

		//DEBUG_TEXT << "====== Point i: " << i << std::endl;

		//cout << "Point i: " << Ci[i].point() << endl;
		int32_t x_coord = round((grid_mapping.x - (double)Ci[i].point().x) / grid_spacing);
		int32_t y_coord = round((grid_mapping.y - (double)Ci[i].point().y) / grid_spacing);

		//FIXME: Implement the function for checking this condition. Better still, learn how to write it as a type independent function!
		ONG_i = Point(matchToRange(x_coord, 0, Nong_x_aux), matchToRange(y_coord, 0, Nong_y_aux));

		//Xi -> ONG:
		//DEBUG_TEXT << "ONG index + Nip1: " << (Nip1 + (ONG_i.y * Nong_x + ONG_i.x)) << endl;
		D.put(Nip1 + (ONG_i.y * Nong_x + ONG_i.x), i, pairwise_occluded_cost_alpha_vel);

		for (int j = 0; j < Nip1; ++j) {
			//DEBUG_TEXT << "   === Point j: " << j << std::endl;
			if (i == 0) {
				//ONG -> Xip1
				//DEBUG_TEXT << "Point " << j << " of ip1 -> x: " << Cip1[j].point() << endl;
				int32_t x_coord2 = round((grid_mapping.x - (double)Cip1[j].point().x) / grid_spacing);
				int32_t y_coord2 = round((grid_mapping.y - (double)Cip1[j].point().y) / grid_spacing);

				//DEBUG_TEXT << "Initial distance: " << x_coord2 << " " << y_coord2 << endl;
				ONG_ip1 = Point(matchToRange(x_coord2, 0, Nong_x_aux), matchToRange(y_coord2, 0, Nong_y_aux));

				//DEBUG_TEXT << "ONG grid matrix index mapping for point " << j << " of frame ip1 is: " << ONG_ip1 << " " << (ONG_ip1.y * Nong_x + ONG_ip1.x) << endl;
				//DEBUG_TEXT << (Ni + (ONG_ip1.y * Nong_x + ONG_ip1.x)) << endl;
				D.put(j, Ni + (ONG_ip1.y * Nong_x + ONG_ip1.x), pairwise_occluded_cost_alpha_vel);
			}

			//Compute distance between points:
			double dx = ((double)Cip1[j].point().x - (double)Ci[i].point().x);
			double dy = ((double)Cip1[j].point().y - (double)Ci[i].point().y);
			double dist = sqrt(dx*dx + dy*dy);
			//DEBUG_TEXT << "Distance:" << Ci[i].point() << " " << Cip1[j].point()<< " " << dist  << endl;

			//Check if it passes the criterion:
			if (dist < max_displacement) {
				//If it passes, put it into D as the inverse distance:
				double inv_dist = 1 - (dist / max_displacement);
				//DEBUG_TEXT << "Transition (" << j << "," << i << ") has distance: " << inv_dist << endl;

				D.put(j, i, inv_dist * alpha_vel);

				if (isinf(inv_dist * alpha_vel)) {
					cout << "Inf detected" << endl;
				}
				if (isnan(inv_dist * alpha_vel)) {
					cout << "NaN detected" << endl;
				}
			}
		}
	}


	//Occluded to Occluded:
	//DEBUG_TEXT << "====== Occluded to occluded. Nong: " << Nong << std::endl;
	for (int i = 0; i < Nong; ++i) {
		//DEBUG_TEXT << "(" << Nip1 + i << "," << Ni + i << ") is occluded to occluded" << endl;
		D.put(Nip1 + i, Ni + i, pairwise_occluded_cost_alpha_vel);
	}

	//DEBUG:
	//M->print_contents("debug_pairwise.txt");

	//Sparsify D before returning:
	//DEBUG_TEXT.close();
	return MATSPARSE(&D);
}


MATSPARSE LocoMouse::pairwisePotential_SideView(const vector<uint> &Zi, const vector<uint> &Zip1, double grid_mapping, double grid_spacing, const vector<uint> &ONGi, const unsigned int Nong, const double max_displacement, const double alpha_vel, const double pairwise_occluded_cost) {

	int Ni = Zi.size();
	int Nip1 = Zip1.size();

	double pairwise_occluded_cost_alpha_vel = pairwise_occluded_cost * alpha_vel;

	int Nong_aux = Nong - 1;

	MyMat D = MyMat(Nip1 + Nong, Ni + Nong);
	//cout << "Size of transition matrix is: " << D.Nrows << " " << D.Ncols << endl;

	uint32_t ONG_i;
	uint32_t ONG_ip1;

	//FIXME: The distance between points could be computed using xDist...

	//cout << "grid_mapping: " << grid_mapping << endl;
	//cout << "grid_spacing: " << grid_spacing << endl;

	//Fi to Fip1/ONG transitions:
	for (int i = 0; i < Ni; ++i) {
		//cout << "Point i: " << Zi[i] << endl;
		int32_t z_coord = round((grid_mapping - (double)Zi[i]) / grid_spacing);

		//FIXME: learn how to write match to range as a type independent function!
		//cout << "Initial coordinates are: " << z_coord << endl;
		ONG_i = matchToRange(z_coord, 0, Nong_aux);
		//cout << "ONG grid matrix index mapping for point " << i << " of frame i is: " << ONG_i << endl;

		//Xi -> ONG: //FIXME: Since we don't have to have a grid mapping here, check if ONG_i is correct!
		D.put(Nip1 + ONG_i, i, pairwise_occluded_cost_alpha_vel);

		for (int j = 0; j < Nip1; ++j) {
			//Compute distance between points:
			double dist = abs((double)Zip1[j] - (double)Zi[i]);
			//cout << "Distance:" << Zi[i] << " " << Zip1[j] << " " << dist << endl;

			//Check if it passes the criterion:
			if (dist < max_displacement) {
				//If it passes, put it into D as the inverse distance:
				double inv_dist = 1 - (dist / max_displacement);
				//cout << "Transition (" << j << "," << i << ") has distance: " << inv_dist << endl;
				D.put(j, i, inv_dist * alpha_vel);
			}
		}
	}

	//ONG -> Xip1
	for (int j = 0; j < Nip1; ++j) {
		//cout << "Point " << j << " of ip1 -> x: " << Zip1[j] << endl;
		int32_t z_coord2 = round((grid_mapping - (double)Zip1[j]) / grid_spacing);

		//cout << "Initial distance: " << z_coord2 << endl;
		ONG_ip1 = matchToRange(z_coord2, 0, Nong_aux);
		//cout << "ONG grid matrix index mapping for point " << j << " of frame ip1 is: " << ONG_ip1 << endl;
		D.put(j, Ni + ONG_ip1, pairwise_occluded_cost_alpha_vel);
	}

	//ONG -> ONG:
	for (int i = 0; i < Nong; ++i) {
		//cout << "(" << Nip1 + i << "," << Ni + i << ") is occluded to occluded" << endl;
		D.put(Nip1 + i, Ni + i, pairwise_occluded_cost_alpha_vel);
	}

	//DEBUG
	/*cout << "--------------" << endl;
	for (int i_cols = 0; i_cols < D.Ncols; i_cols++) {
	for (int i_rows = 0; i_rows < D.Nrows; i_rows++) {
	if (D.get(i_rows, i_cols) != 0) {
	cout << "D(" << i_rows << "," << i_cols << ") is " << D.get(i_rows, i_cols) << endl;
	}
	}
	}*/

	//Sparsify D before returning:
	return MATSPARSE(&D);
}


void LocoMouse::computeBottomTracks() {
	/*	Calls the match2nd routine for solving the most likely path for the tracks

		Since the algorithm is order dependent, we try all 24 permutations for the paws.
		The snout only needs to be run once, as it is a single feature.

	*/

	//Tracking the paws for all possible permutations:
	cv::Mat M_permutation;
	double current_cost = -1;
	unsigned int current_perm = 0;

	for (unsigned int i_perm = 0; i_perm < LM_PARAMS.N_paws; i_perm++) {
		const int *pOrder = LM_PARAMS.PAW_PERMUTATIONS.ptr<int>(i_perm);

		cv::Mat M_i = match2nd(UNARY_BOTTOM_PAW, PAIRWISE_BOTTOM_PAW, ONG.size(), 0, 0, N_FRAMES, LM_PARAMS.N_paws, pOrder);

		double c_i = computeCostTrack(M_i, UNARY_BOTTOM_PAW, PAIRWISE_BOTTOM_PAW, pOrder);

		if (c_i > current_cost) {
			current_perm = i_perm;
			current_cost = c_i;
			M_i.copyTo(M_permutation);
		}
	}

	//Reverting the permutations:
	TRACK_INDEX_PAW_BOTTOM = Mat::zeros(M_permutation.rows, M_permutation.cols, M_permutation.type());
	const int *pOrder = LM_PARAMS.PAW_PERMUTATIONS.ptr<int>(current_perm);
	for (int i_row = 0; i_row < 4; i_row++) {
		M_permutation.row(i_row).copyTo(TRACK_INDEX_PAW_BOTTOM.row(pOrder[i_row]));
	}

	//Tracking the snout:
	int a = 0;
	int* porder = &a;
	TRACK_INDEX_SNOUT_BOTTOM = match2nd(UNARY_BOTTOM_SNOUT, PAIRWISE_BOTTOM_SNOUT, ONG.size(), 0, 0, N_FRAMES, LM_PARAMS.N_snout, porder);

	return;
}

void LocoMouse::computeSideTracks() {

	TRACK_INDEX_PAW_SIDE = bestSideViewMatch(TRACK_INDEX_PAW_BOTTOM, CANDIDATES_MATCHED_VIEWS_PAW, ONG_SIDE, ONG_SIDE_LOWEST_POINT, LM_PARAMS.N_paws);

	TRACK_INDEX_SNOUT_SIDE = bestSideViewMatch(TRACK_INDEX_SNOUT_BOTTOM, CANDIDATES_MATCHED_VIEWS_SNOUT, ONG_SIDE, ONG_SIDE_LOWEST_POINT, LM_PARAMS.N_snout);

	return;
}


cv::Mat LocoMouse::bestSideViewMatch(const Mat& T, const vector< vector<P22D> > &candidates_bottom_top_matched, const vector<uint> &ONG_side, const unsigned int lowest_point, const unsigned int N_features) {

	//FIXME: Properly set the debug flag
	bool debug = false;

	if (debug) {
		DEBUG_TEXT.open("debug_side_tracks.txt");
	}

	int Nong_top = ONG_side.size();

	bool change_pointer_T = true;
	if (T.isContinuous()) {
		change_pointer_T = false;
	}
	const int* p_T = T.ptr<int>(0);

	cv::Mat T_side(N_features, N_FRAMES, CV_32SC1); //Output matrix;

	//Define the auxiliary unary and pairwise vectors:
	//FIXME: These could be defined at the top.

	for (unsigned int i_feature = 0; i_feature < N_features; i_feature++) {

		if (debug) {
			DEBUG_TEXT << "Computing side view tracks for feature: " << i_feature << endl;
		}

		vector<unsigned int> Z_prev;
		//vector <MyMat> unary_potential_top(N_frames);
		//vector <MATSPARSE> pairwise_potential_top(N_frames - 1);

		vector <MyMat> unary_potential_side;
		unary_potential_side.reserve(N_FRAMES);
		vector <MATSPARSE> pairwise_potential_side;
		pairwise_potential_side.reserve(N_FRAMES - 1);

		if (i_feature > 0) {
			if (change_pointer_T) {
				p_T = T.ptr<int>(i_feature);
			}
			else {
				p_T += N_FRAMES;
			}
		}

		//Check Bottom view results:
		for (int i_frames = 0; i_frames < N_FRAMES; i_frames++) {
			vector<uint> Z;
			MyMat frame_potentials;

			if (debug) {
				DEBUG_TEXT << i_frames << endl;
			}

			if (p_T[i_frames] < candidates_bottom_top_matched[i_frames].size()) {
				//There is a solution on the bottom view. Check for candidates on the side view:

				frame_potentials = MyMat(candidates_bottom_top_matched[i_frames][p_T[i_frames]].number_of_candidates(), 1);

				for (int i_top = 0; i_top < candidates_bottom_top_matched[i_frames][p_T[i_frames]].number_of_candidates(); i_top++) {
					//Copying the scores of top candidates

					frame_potentials.put(i_top, 0, candidates_bottom_top_matched[i_frames][p_T[i_frames]].score_top(i_top));
					Z.push_back(candidates_bottom_top_matched[i_frames][p_T[i_frames]].y_top_coord(i_top));

				}

				if (debug)
					DEBUG_TEXT << "Pushed top candidates" << endl;

			}
			else {
				//There is no solution on the bottom view. Add occlusion grid points only.
				frame_potentials = MyMat(0, 1);

				if (debug)
					DEBUG_TEXT << "No top candidates" << endl;

			}

			//FIXME: Check types, its complaining.
			unary_potential_side.push_back(frame_potentials);

			if (debug)
				DEBUG_TEXT << "Pushed potentials" << endl;

			if (i_frames > 0) {
				//DEBUG_TEXT << "Computing pairwise potentials: " << endl;
				//Compute pairwise costs for the top view:

				MATSPARSE temp_matsparse = pairwisePotential_SideView(Z_prev, Z, (double)lowest_point, (double)LM_PARAMS.occlusion_grid_spacing_pixels, ONG_side, Nong_top, LM_PARAMS.max_displacement_top, LM_PARAMS.alpha_vel_top, LM_PARAMS.pairwise_occluded_cost);
				pairwise_potential_side.push_back(temp_matsparse);

				if (debug)
					DEBUG_TEXT << "Pushed pairwise potentials" << endl;

			}

			Z_prev = Z;
		}

		//Solve match2nd tracking for the current top view:
		int a = 0;
		int* porder = &a;
		if (debug)
			DEBUG_TEXT << "Running match2nd on top view..." << endl;

		//FIXME: Might be wrong depending on how the smart pointer is handeled.

		cv::Mat temp = match2nd(unary_potential_side, pairwise_potential_side, Nong_top, 0, 0, N_FRAMES, 1, porder);

		temp.copyTo(T_side.row(i_feature));

		if (debug) {
			DEBUG_TEXT << "Done" << endl;
			DEBUG_TEXT.close();
		}
	}

	return T_side;
}

void LocoMouse::exportResults() {

	//Export Paw Tracks:
	exportPointTracks(TRACK_INDEX_PAW_BOTTOM, TRACK_INDEX_PAW_SIDE, CANDIDATES_MATCHED_VIEWS_PAW, "paw_tracks", LM_PARAMS.N_paws);

	//Export Snout Tracks:
	exportPointTracks(TRACK_INDEX_SNOUT_BOTTOM, TRACK_INDEX_SNOUT_SIDE, CANDIDATES_MATCHED_VIEWS_SNOUT, "snout_tracks", LM_PARAMS.N_snout);

	//Export Tail Tracks:
	exportLineTracks(TRACKS_TAIL, "tracks_tail", LM_PARAMS.N_tail_points);

	return;
}

void LocoMouse::exportPointTracks(const Mat& T_bottom, const Mat &T_top, const vector< vector<P22D> > &candidates_bottom_top_matched, const string feature_name, const unsigned int N_features) {

	bool change_pointer_M = true;
	if (T_bottom.isContinuous()) {
		change_pointer_M = false;
	}

	const int* p_T = T_bottom.ptr<int>(0);

	for (int i_features = 0; i_features < N_features; ++i_features) {

		if (i_features > 0) {
			if (change_pointer_M)
				p_T = T_bottom.ptr<int>(i_features);
			else
				p_T += N_FRAMES;
		}

		//Pointer for side view match2nd results for paw i_paws
		const int* p_T_top = T_top.ptr<int>(i_features);

		//Not sure initializing to ones and negating is faster, but whatever.
		Mat M = -cv::Mat::ones(N_FRAMES, 3, CV_32SC1);

		bool change_pointer_M = true;
		if (M.isContinuous())
			change_pointer_M = false;

		//
		int* p_M = M.ptr<int>(0);

		for (int i_frames = 0; i_frames < N_FRAMES; ++i_frames) {
			//DEBUG_TEXT << "Output frames: " << i_frames << endl;
			if (p_T[i_frames] < candidates_bottom_top_matched[i_frames].size()) {
				//debug << "Feature " << i_features << ", Frame " << i_frames << ", coordinates: " << candidates_bottom_top_matched[i_frames][p_T[i_frames]].x_coord() << ", " << candidates_bottom_top_matched[i_frames][p_T[i_frames]].y_bottom_coord() << endl;

				p_M[0] = BB_X_POS[i_frames] - BB_BOTTOM_MOUSE.width + 1 + candidates_bottom_top_matched[i_frames][p_T[i_frames]].x_coord();
				p_M[1] = BB_Y_BOTTOM_POS[i_frames] - BB_BOTTOM_MOUSE.height + 1 + candidates_bottom_top_matched[i_frames][p_T[i_frames]].y_bottom_coord();

				/*if (i_features == 1 & i_frames == 12) {
				cout << "M_bottom: " << p_T[i_frames] << endl;
				cout << "M_value: " << p_T_top[i_frames] << endl;
				cout << "N candidates: " << candidates_bottom_top_matched[i_frames][p_T[i_frames]].number_of_candidates() << endl;
				}*/

				if (p_T_top[i_frames] < candidates_bottom_top_matched[i_frames][p_T[i_frames]].number_of_candidates()) {

					p_M[2] = BB_Y_SIDE_POS[i_frames] - BB_SIDE_MOUSE.height + 1 + candidates_bottom_top_matched[i_frames][p_T[i_frames]].y_top_coord(p_T_top[i_frames]);

				}

			}

			if (change_pointer_M)
				p_M = M.ptr<int>(i_frames + 1);
			else
				p_M += 3;
		}

		stringstream s;
		s << feature_name << i_features;
		string temp;
		s >> temp;
		OUTPUT << temp << M;
	}

	return;
}

void LocoMouse::exportLineTracks(std::vector <cv::Mat> Tracks, std::string track_name, int N_line_points) {

	cv::Mat LineTracksFinal = -1 * Mat::ones(3, N_line_points * N_FRAMES, CV_32SC1);

	int* tail_x = LineTracksFinal.ptr<int>(0);
	int* tail_y = LineTracksFinal.ptr<int>(1);
	int* tail_z = LineTracksFinal.ptr<int>(2);

	for (unsigned int i = 0; i < N_FRAMES; ++i) {

		for (int track = 0; track < N_line_points; ++track, tail_x += 1, tail_y += 1, tail_z += 1) {

			if (Tracks[i].at<int>(0, track) >= 0)
				tail_x[0] = BB_X_POS[i] - BB_BOTTOM_MOUSE.width + 1 + Tracks[i].at<int>(0, track);

			//Note: Not adding bottom_view.y anymore.
			if (Tracks[i].at<int>(1, track) >= 0)
				tail_y[0] = BB_Y_BOTTOM_POS[i] - BB_BOTTOM_MOUSE.height + 1 + Tracks[i].at<int>(1, track);

			if (Tracks[i].at<int>(2, track) >= 0) {
				tail_z[0] = BB_Y_SIDE_POS[i] - BB_SIDE_MOUSE.height + 1 + Tracks[i].at<int>(2, track);
			}
		}
	}

	OUTPUT << track_name << LineTracksFinal;

	return;
}



void LocoMouse::exportTracksBottom(const Mat& T_bottom, const vector< vector <Candidate> > &candidates_bottom, const string feature_name, const unsigned int N_features) {

	bool change_pointer_M = true;
	if (T_bottom.isContinuous()) {
		change_pointer_M = false;
	}

	const int* p_T = T_bottom.ptr<int>(0);

	for (int i_features = 0; i_features < N_features; ++i_features) {

		if (i_features > 0) {
			if (change_pointer_M)
				p_T = T_bottom.ptr<int>(i_features);
			else
				p_T += N_FRAMES;
		}

		//Pointer for side view match2nd results for paw i_paws
		//const int* p_T_top = T_top[i_features].ptr<int>(0);

		cv::Mat M = -cv::Mat::ones(N_FRAMES, 3, CV_32SC1);

		bool change_pointer_M = true;

		if (M.isContinuous()) {
			change_pointer_M = false;
		}

		int* p_M = M.ptr<int>(0);

		for (int i_frames = 0; i_frames < N_FRAMES; ++i_frames) {

			if (p_T[i_frames] < candidates_bottom[i_frames].size()) {

				p_M[0] = BB_X_POS[i_frames] - BB_BOTTOM_MOUSE.width + 1 + candidates_bottom[i_frames][p_T[i_frames]].point().x;
				p_M[1] = BB_Y_BOTTOM_POS[i_frames] - BB_BOTTOM_MOUSE.height + 1 + candidates_bottom[i_frames][p_T[i_frames]].point().y;
			}

			if (change_pointer_M)
				p_M = M.ptr<int>(i_frames + 1);
			else
				p_M += 3;
		}

		stringstream s;
		s << feature_name << i_features;
		string temp;
		s >> temp;
		OUTPUT << temp << M;
	}

	return;
}

void LocoMouse::detectTail() {

	TRACKS_TAIL.push_back(detectLineCandidates(M.tail, LM_PARAMS.N_tail_points, TAIL_MASK));

	return;

}


cv::Mat LocoMouse::detectLineCandidates(const LocoMouse_Feature F, const unsigned int N_line_points, cv::Mat& TAIL_MASK) {
	//Tail detection using the detector and a connected components algorithm over the detection.

	//FIXME: Implement proper debugging:
	bool debug = false;
	std::ofstream DEBUG_TEXT;
	cv::FileStorage debug_locomouse;

	if (debug) {
		DEBUG_TEXT.open("debug_tail.txt");
		debug_locomouse.open("debug_tail.yml", cv::FileStorage::WRITE);

		DEBUG_TEXT << "------ detectTail ------" << endl;
	}

	cv::Mat I_tail_bb_bottom_pad, I_tail_bb_side_pad;
	I_tail_bb_bottom_pad = I_BOTTOM_MOUSE_PAD(BB_BOTTOM_TAIL_PAD);
	I_tail_bb_side_pad = I_SIDE_MOUSE_PAD(BB_SIDE_TAIL_PAD);

	if (debug) {
		DEBUG_TEXT << "Cropping I_tail_pad done (both views)" << endl;
	}

	cv::Mat Filtered_tail_bottom_pad, Filtered_tail_side_pad, Filtered_tail_bottom, Filtered_tail_side;
	//Searching for the tail:
	cv::filter2D(I_tail_bb_bottom_pad, Filtered_tail_bottom_pad, CV_32F, F.detector_bottom(), Point(-1, -1), -F.rho_bottom(), BORDER_CONSTANT);
	cv::filter2D(I_tail_bb_side_pad, Filtered_tail_side_pad, CV_32F, F.detector_side(), Point(-1, -1), -F.rho_side(), BORDER_CONSTANT);

	if (debug) {
		DEBUG_TEXT << "Filtering tail done (both views)" << endl;
		debug_locomouse << "Filtered_tail_bottom_pad" << Filtered_tail_bottom_pad;
		debug_locomouse << "Filtered_tail_top_pad" << Filtered_tail_side_pad;
	}

	//Unpadding the tail image:
	Filtered_tail_bottom = Filtered_tail_bottom_pad(BB_UNPAD_TAIL_BOTTOM);
	Filtered_tail_side = Filtered_tail_side_pad(BB_UNPAD_TAIL_SIDE);

	if (debug) {
		DEBUG_TEXT << "Unpadding filtering done (both views)." << endl;
	}

	//Converting to [0,1] range for binarization
	cv::threshold(Filtered_tail_bottom, Filtered_tail_bottom, 0, 1, THRESH_BINARY);
	cv::threshold(Filtered_tail_side, Filtered_tail_side, 0, 1, THRESH_BINARY); //FIXME: This step might not be needed.

	//Image must be CV_8UC1 to perform the connected component analysis
	cv::Mat Filtered_tail_bottom_binary;
	Filtered_tail_bottom.convertTo(Filtered_tail_bottom_binary, CV_8UC1);

	//Dividing the tail into a predefined number of segments:
	cv::Mat TailTracks = -cv::Mat::ones(3, N_line_points, CV_32SC1);

	//Searching for the largest area object and considering it to be the tail
	selectLargestRegion(Filtered_tail_bottom_binary, Filtered_tail_bottom_binary);

	if (debug) {
		DEBUG_TEXT << "Select largest bottom region done." << endl;
	}

	//Mask for removing tail interference on bottom view paw tracking:
	TAIL_MASK = Filtered_tail_bottom_binary;
	//cv::Mat Filtered_tail_bottom_binary_reversed = (Filtered_tail_bottom_binary == 0);
	//Filtered_tail_bottom_binary_reversed.convertTo(Filtered_tail_bottom, CV_32FC1); //Converting to double to multiply against filter results.
	//normalize(Filtered_tail_bottom, Filtered_tail_bottom, 0, 1, NORM_MINMAX, CV_32FC1);

	//Masking the bottom region on the top view:
	cv::Mat tail_bottom_x_location;
	cv::reduce(Filtered_tail_bottom_binary, tail_bottom_x_location, 0, CV_REDUCE_MAX);
	cv::Mat tail_mask_top = cv::repeat(tail_bottom_x_location, Filtered_tail_side.rows, 1);
	cv::Mat Filtered_tail_top_binary = (Filtered_tail_side > 0) & tail_mask_top;

	selectLargestRegion(Filtered_tail_top_binary, Filtered_tail_top_binary);

	if (debug) {
		DEBUG_TEXT << "Select largest top region done." << endl;
	}

	//FIXME: Code a function for first_last that works with different matrix types:
	int first_last[2] = { 0,0 };
	bool no_first = true;

	uchar* p_tail_bottom = tail_bottom_x_location.ptr<uchar>(0);

	if (debug) {
		debug_locomouse << "tail_bottom_x_location" << tail_bottom_x_location;
	}

	for (int i = 0; i < tail_bottom_x_location.cols; ++i) {
		if (p_tail_bottom[i] > 0) {
			first_last[0] = i;
			no_first = false;
			break;
		}
	}

	if (debug) {
		DEBUG_TEXT << "First point is: " << first_last[0] << " out of " << tail_bottom_x_location.cols << endl;
	}

	if (no_first) {

		if (debug) {
			DEBUG_TEXT << "No tail region found. Returning -1." << endl;
		}

		return TailTracks;
	}
	else {

		first_last[1] = first_last[0];
		for (int i = tail_bottom_x_location.cols - 1; i > first_last[0]; i--) {
			if (p_tail_bottom[i] > 0) {
				first_last[1] = i;
				break;
			}
		}

		if (debug) {
			DEBUG_TEXT << "First and last: " << first_last[0] << " " << first_last[1] << endl;
		}

		//Consider having this as a function on some sort of personal library.
		int tail_width = (first_last[1] - first_last[0]);
		int remainder = tail_width % N_line_points;
		int regular_length = (tail_width - remainder) / N_line_points;

		std::vector <int> bb_tail_segment_bottom_width(N_line_points, regular_length);

		for (unsigned int i = 0; i < remainder; i++) {
			bb_tail_segment_bottom_width[i] = regular_length + 1;
		}

		cv::Rect BB_tail_segment_bottom = cv::Rect(first_last[0], 0, 1, Filtered_tail_bottom_binary.rows);
		cv::Rect BB_tail_segment_top = cv::Rect(first_last[0], 0, 1, Filtered_tail_top_binary.rows);

		int* p_tail_x = TailTracks.ptr<int>(0);
		int* p_tail_y = TailTracks.ptr<int>(1);
		int* p_tail_z = TailTracks.ptr<int>(2);

		{
			cv::Mat Itail_segment, contours, hierarchy;
			cv::Moments M;

			//Tail bottom:
			if (debug) {
				DEBUG_TEXT << "--- Tail segments ---" << endl;
			}
			for (int i = 0; i < N_line_points; i++) {

				if (debug) {
					DEBUG_TEXT << "Filtered_tail_bottom_binary size: " << BB_tail_segment_bottom << endl;
					DEBUG_TEXT << "bb_tail_segment_bottom_width: " << bb_tail_segment_bottom_width[i] << endl;
				}

				BB_tail_segment_bottom.width = bb_tail_segment_bottom_width[i];

				Itail_segment = Filtered_tail_bottom_binary(BB_tail_segment_bottom);

				if (debug) {
					DEBUG_TEXT << "Crop segment done." << endl;
				}

				M = moments(Itail_segment, true);
				if (M.m00 > 0) {
					p_tail_x[i] = (int)(M.m10 / M.m00) + BB_tail_segment_bottom.x;
					p_tail_y[i] = (int)(M.m01 / M.m00) + BB_tail_segment_bottom.y;
				}

				//Move the box:
				BB_tail_segment_bottom += Point(BB_tail_segment_bottom.width, 0);
			}

			//Tail top:
			for (int i = 0; i < N_line_points; i++) {
				if (p_tail_x[i] > 0) {
					BB_tail_segment_top.x = p_tail_x[i];

					Itail_segment = Filtered_tail_top_binary(BB_tail_segment_top);
					M = cv::moments(Itail_segment, true);
					if (M.m00 > 0)
						p_tail_z[i] = (int)(M.m01 / M.m00) + BB_tail_segment_top.y;
				}
			}
		}

		return TailTracks;
	}
}

void LocoMouse::selectLargestRegion(const cv::Mat &Iin, cv::Mat &Iout) {

	cv::Mat labels, stats, centroids;
	int N_labels = connectedComponentsWithStats(Iin, labels, stats, centroids, LM_PARAMS.conn_comp_connectivity, CV_16U);
	if (N_labels > 1) {
		//OpenCV assumes the first object to be the background.t
		int largest_object = 1;
		int largest_object_area = stats.at<int>(1, CC_STAT_AREA);
		for (int i_labels = 2; i_labels < N_labels; ++i_labels) {
			if (stats.at<int>(i_labels, CC_STAT_AREA) > largest_object_area) {
				largest_object = i_labels;
				largest_object_area = stats.at<int>(i_labels, CC_STAT_AREA);
			}
		}

		//Selecting the largest object: 
		compare(labels, largest_object, Iout, CMP_EQ); //FIXME: This is made for masking the tail not for getting its position!!
	}
	else {
		Iout = cv::Mat::zeros(Iin.size(), CV_8UC1);
	}
}




unsigned int LocoMouse::N_frames() const {

	return N_FRAMES;

}


//----- LocoMouse_Feature:
LocoMouse_Feature::LocoMouse_Feature() : w_b(cv::Mat::zeros(cv::Size(1, 1), CV_8UC1)), w_s(cv::Mat::zeros(cv::Size(1, 1), CV_8UC1)), rho_b(0), rho_s(0) {

	size_b = w_b.size();
	size_s = w_s.size();
	match_rect_b = cv::Rect(0, 0, 1, 1);
	match_rect_s = cv::Rect(0, 0, 1, 1);
	w_avg_i = cv::Mat::zeros(cv::Size(1, 1), CV_64FC1);
	w_avg_j = cv::Mat::zeros(cv::Size(1, 1), CV_64FC1);
};

LocoMouse_Feature::LocoMouse_Feature(cv::Mat window_bottom, cv::Mat window_top, double bias_bottom, double bias_top) {

	w_b = window_bottom;
	w_s = window_top;
	rho_b = bias_bottom;
	rho_s = bias_top;

	size_b = window_bottom.size();
	size_s = window_top.size();

	//Velocity Matching Mask is defined as a mask with half the size of the detector.
	//The mask is built so that Mask + Point centers the mask at the point coordinates.

	int new_b_w = round(((double)w_b.cols) / 2);
	int new_b_h = round(((double)w_b.rows) / 2);

	int new_t_w = round(((double)w_s.cols) / 2);
	int new_t_h = round(((double)w_s.rows) / 2);

	int new_bottom_tlx, new_bottom_tly, new_side_tlx, new_side_tly;

	new_bottom_tlx = new_b_w / 2;
	new_bottom_tly = new_b_h / 2;

	new_side_tlx = new_t_w / 2;
	new_side_tly = new_t_h / 2;

	match_rect_b = cv::Rect(-new_bottom_tlx, -new_bottom_tly, new_b_w, new_b_h);
	match_rect_s = cv::Rect(-new_side_tlx, -new_side_tly, new_t_w, new_t_h);

	w_avg_i = cv::Mat::zeros(cv::Size(w_b.rows, w_b.cols), CV_32FC1);
	w_avg_j = cv::Mat::zeros(cv::Size(w_b.rows, w_b.cols), CV_32FC1);

	//Writing column and row weights template:

	for (int i_rows = 0; i_rows < w_b.rows; ++i_rows) {

		float* p_wavg_i = w_avg_i.ptr<float>(i_rows);
		float* p_wavg_j = w_avg_j.ptr<float>(i_rows);

		for (int i_cols = 0; i_cols < w_b.cols; ++i_cols) {
			p_wavg_i[i_cols] = i_rows;
			p_wavg_j[i_cols] = i_cols;
		}
	}
	//They are not the transpose of each other if they are not square. 
	w_avg_i = w_avg_i - w_b.rows / 2;
	w_avg_j = w_avg_j - w_b.cols / 2;

}

LocoMouse_Feature& LocoMouse_Feature::operator=(LocoMouse_Feature& other) {
	//Move assign:

	w_b = other.w_b;
	w_s = other.w_s;
	rho_b = other.rho_b;
	rho_s = other.rho_s;

	size_b = other.size_b;
	size_s = other.size_s;

	match_rect_b = other.match_rect_b;
	match_rect_s = other.match_rect_s;

	w_avg_i = other.w_avg_i;
	w_avg_j = other.w_avg_j;

	return *this;

}

cv::Mat LocoMouse_Feature::detector_bottom() const {
	return w_b;
}

cv::Mat LocoMouse_Feature::detector_side() const {
	return w_s;
}

double LocoMouse_Feature::rho_bottom() const {
	return rho_b;
}

double LocoMouse_Feature::rho_side() const {
	return rho_s;
}

cv::Size LocoMouse_Feature::size_bottom() const {
	return size_b;
}

cv::Size LocoMouse_Feature::size_side() const {
	return size_s;
}

cv::Mat LocoMouse_Feature::weighted_average_i() const {
	return w_avg_i;
};
cv::Mat LocoMouse_Feature::weighted_average_j() const {
	return w_avg_j;
};

cv::Rect LocoMouse_Feature::match_box_bottom() const {
	return match_rect_b;
}

cv::Rect LocoMouse_Feature::match_box_side() const {
	return match_rect_s;
}

//class LocoMouse_Model
LocoMouse_Model::LocoMouse_Model() {

	cv::Mat Z = cv::Mat::zeros(1, 1, CV_64FC1);

	paw = LocoMouse_Feature(Z, Z, 0, 0);
	snout = LocoMouse_Feature(Z, Z, 0, 0);
	tail = LocoMouse_Feature(Z, Z, 0, 0);

	//Pre and Post are properties of the set of models:
	spre_t = cv::Size(0, 0);
	spre_b = cv::Size(0, 0);

	spost_t = cv::Size(0, 0);
	spost_b = cv::Size(0, 0);
}

LocoMouse_Model::~LocoMouse_Model() {
	//Nothing to release as opencv stuff is implemented with smart pointers.
}

LocoMouse_Model::LocoMouse_Model(std::string file_name) {

	cv::FileStorage model_file(file_name, cv::FileStorage::READ);
	if (!model_file.isOpened()) {
		throw std::invalid_argument(Stream_Formatter() << "Error: Could not open the model file: " << file_name << '\n');
	}

	cv::Mat w_paw_b, w_paw_t, w_snout_b, w_snout_t, w_tail_b, w_tail_t;
	double rho_paw_b, rho_paw_t, rho_snout_b, rho_snout_t, rho_tail_b, rho_tail_t;

	//Read matrices:
	//FIXME: Find a way to throw an exception here!
	model_file["modelPaw_top"] >> w_paw_t;
	if (w_paw_t.empty()) { 
		model_file.release();
		throw std::invalid_argument(Stream_Formatter() << "Error: modelPaw_top cannot be empty." << file_name << '\n');
	};

	model_file["modelPaw_bottom"] >> w_paw_b;
	if (w_paw_t.empty()) {
		model_file.release();
		throw std::invalid_argument(Stream_Formatter() << "Error: modelPaw_bottom cannot be empty." << file_name << '\n');
	};

	model_file["modelTail_top"] >> w_tail_t;
	if (w_paw_t.empty()) {
		model_file.release();
		throw std::invalid_argument(Stream_Formatter() << "Error: modelTail_top cannot be empty." << file_name << '\n');
	};

	model_file["modelTail_bottom"] >> w_tail_b;
	if (w_paw_t.empty()) {
		model_file.release();
		throw std::invalid_argument(Stream_Formatter() << "Error: modelTail_bottom cannot be empty." << file_name << '\n');
	};

	model_file["modelSnout_top"] >> w_snout_t;
	if (w_paw_t.empty()) {
		model_file.release();
		throw std::invalid_argument(Stream_Formatter() << "Error: modelSnout_top cannot be empty." << file_name << '\n');
	};

	model_file["modelSnout_bottom"] >> w_snout_b;
	if (w_paw_t.empty()) {
		model_file.release();
		throw std::invalid_argument(Stream_Formatter() << "Error: modelSnout_bottom cannot be empty." << file_name << '\n');
	};

	//FIXME: How to check if these exist? If not defined will they be set to 0?
	model_file["biasPaw_top"] >> rho_paw_t;
	model_file["biasPaw_bottom"] >> rho_paw_b;
	model_file["biasTail_top"] >> rho_tail_t;
	model_file["biasTail_bottom"] >> rho_tail_b;
	model_file["biasSnout_top"] >> rho_snout_t;
	model_file["biasSnout_bottom"] >> rho_snout_b;

	model_file.release();

	paw = LocoMouse_Feature(w_paw_b, w_paw_t, rho_paw_b, rho_paw_t);
	snout = LocoMouse_Feature(w_snout_b, w_snout_t, rho_snout_b, rho_snout_t);
	tail = LocoMouse_Feature(w_tail_b, w_tail_t, rho_tail_b, rho_tail_t);

	//Pre and Post are properties of the set of models:
	spre_t = cv::Size(ceil((double)(std::max(w_paw_t.cols, w_snout_t.cols) - 1) / 2), ceil((double)(std::max(w_paw_t.rows, w_snout_t.rows) - 1) / 2));
	spre_b = cv::Size(ceil((double)(std::max(w_paw_b.cols, w_snout_b.cols) - 1) / 2), ceil((double)(std::max(w_paw_b.rows, w_snout_b.rows) - 1) / 2));

	spost_t = cv::Size((std::max(w_paw_t.cols, w_snout_t.cols) - 1) / 2, (std::max(w_paw_t.rows, w_snout_t.rows) - 1) / 2);
	spost_b = cv::Size((std::max(w_paw_b.cols, w_snout_b.cols) - 1) / 2, (std::max(w_paw_b.rows, w_snout_b.rows) - 1) / 2);
}

LocoMouse_Model& LocoMouse_Model::operator=(LocoMouse_Model &other) {
	//Move assign operator:

	//Reassigning the data:
	paw = other.paw;
	snout = other.snout;
	tail = other.tail;

	spre_b = other.size_pre_bottom();
	spost_b = other.size_pre_bottom();

	spre_t = other.size_pre_side();
	spost_t = other.size_post_side();

	return *this;
}


//Appending sizes:
cv::Size LocoMouse_Model::size_pre_bottom() const { return spre_b; };
cv::Size LocoMouse_Model::size_pre_side() const { return spre_t; };

cv::Size LocoMouse_Model::size_post_bottom() const { return spost_b; };
cv::Size LocoMouse_Model::size_post_side() const { return spost_t; };

//class LocoMouse_LocationPrior:
LocoMouse_LocationPrior::LocoMouse_LocationPrior() {
	POSITION = cv::Point_<double>(0, 0);
	AREA = cv::Rect_<double>(0, 0, 1, 1);
	MAX_DISTANCE = 1;
}

LocoMouse_LocationPrior::LocoMouse_LocationPrior(double x, double y, double md, double minx, double maxx, double miny, double maxy) {
	CV_Assert(minx < maxx);
	CV_Assert(miny < maxy);
	POSITION = cv::Point_<double>(x, y);
	AREA = cv::Rect_<double>(minx, miny, maxx - minx, maxy - miny);
	MAX_DISTANCE = md;
};

//AUX Functions. Shall this be in different h/cpp files?
template <typename T>
T matchToRange(T input, T min_val, T max_val) {
	if (input < min_val) {
		return min_val;
	}
	else if (input > max_val) {
		return max_val;
	}

	return input;
}
