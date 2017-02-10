/* LocoMouse C++ Version

	Written by Joao Fayad (joaofayad@gmail.com)

*/

//OpenCV includes:
//#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/highgui.hpp>

//Other C++ includes:
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <vector>
#include <numeric>
#include <functional>
#include <map>
#include <exception>

//LocoMouse includes:
#include "MyMat.hpp"
#include "Candidates.hpp"
#include "match2nd.h"

//FIXME: New classes merge with other classes
#include "LocoMouse_class.hpp"

//std::ofstream debug_text;
//cv::FileStorage debug_locomouse;

//int parseInputs(int argc, char* argv[]);

int main(int argc, char* argv[]) {
	double t = (double) getTickCount();

	try {

		//FIXME: Implement this properly after debug:
	/*	debug_text.open("debug.txt");
		debug_locomouse.open("debug_locomouse.yml", cv::FileStorage::WRITE);
		
		debug_text << "Start debug" << std::endl;
		debug_text << "Using OpenCV version: " << CV_VERSION << std::endl;
*/
		//Initialize LocoMouse problem
		LocoMouse L(argc, argv);

		//Compute Bounding Box location accross video:
		L.computeBoundingBox();
		//debug_text << "Bounding box computed" << std::endl;

		L.initializeFeatureLoop();
		//debug_text << "Initialized feature loop " << std::endl;


		//Compute per-frame candidate locations:
		for (unsigned int i_frames = 0; i_frames < L.N_frames(); ++i_frames) {
			//debug_text << "Current frame: " << i_frames << std::endl;
			
			//Read new image:
			L.readFrame();
			//debug_text << "Read Frame" << std::endl;

			//Crop image bounding boxes for both views:
			L.cropBoundingBox();
			//debug_text << "Cropped Box" << std::endl;

			// --- Detecting the Tail:
			L.detectTail();
			//debug_text << "Tail" << std::endl;

			// --- Detecting Paw + Snout candidates for bottom view:
			L.detectBottomCandidates();
			//debug_text << "Bottom Candidates" << std::endl;

			// --- Computing Unary costs based on Location Prior:
			L.computeUnaryCostsBottom();
			//debug_text << "Unary Cost" << std::endl;

			// --- Computing Pairwise costs (Displacement between frames):
			L.computePairwiseCostsBottom();
			//debug_text << "Pairwise Cost" << std::endl;

			// --- Detecting Paw + Snout candidates for side view:
			L.detectSideCandidates();
			//debug_text << "Detect Side Candidates" << std::endl;
			
			// --- Matching 
			L.matchBottomSideCandidates();
			//debug_text << "Match Bottom and Side Views" << std::endl;

			L.storePreviousImage();

		}

		//--- Paws + Snout tracks:
		L.computeBottomTracks();
		//debug_text << "Bottom tracks done" << std::endl;

		////Estimate corresponding Z coordinates:
		L.computeSideTracks();
		//debug_text << "Side tracks done." << std::endl;

		//--- Export results:
		L.exportResults();
		//debug_text << "Export done." << std::endl;
	}
	catch (std::invalid_argument e) {
		std::cout << "Invalid inputs: " << e.what() << std::endl;
	}
	catch (std::runtime_error e) {
		std::cout << "Runtime Error: " << e.what() << std::endl;
	}

	//debug_text.close();
	//debug_locomouse.release();
	t = ((double)getTickCount() - t) / getTickFrequency();
	std::cout << "Total Elapsed time: " << t << "s" << std::endl;
	return EXIT_SUCCESS;
}

//int parseInputs(int argc, char* argv[]) {
//
//	std::string LM_CALL, CONFIG_FILE, VIDEO_FILE, BKG_FILE, MODEL_FILE, CALIBRATION_FILE, FLIP_CHAR, OUTPUT_PATH, REF_PATH;
//	
//	if (argc != 8) {
//		std::cout << "Warning: Invalid input list. The input should be: LocoMouse config.yml video.avi background.png model_file.yml calibration_file.yml side_char output_folder." << std::endl;
//		std::cout << "Attempting to run with default paramters..." << std::endl;
//
//		LM_CALL = std::string(argv[0]);
//		VIDEO_FILE = "temp.avi";
//
//		configurePath();
//
//		CONFIG_FILE = REF_PATH + "config.yml";
//		VIDEO_FILE = REF_PATH + "L7Y9_control1_L.avi";
//		BKG_FILE = REF_PATH + "L7Y9_control1_L.png";
//		MODEL_FILE = REF_PATH + "model_LocoMouse_paper.yml";
//		CALIBRATION_FILE = REF_PATH + "IDX_pen_correct_fields2.yml";
//		FLIP_CHAR = "L";
//		OUTPUT_PATH = REF_PATH + ".";
//
//	}
//	else {
//		LM_CALL = std::string(argv[0]);
//		CONFIG_FILE = std::string(argv[1]);
//		VIDEO_FILE = std::string(argv[2]);
//		MODEL_FILE = std::string(argv[4]);
//		BKG_FILE = std::string(argv[3]);
//		CALIBRATION_FILE = std::string(argv[5]);
//		FLIP_CHAR = std::string(argv[6]);
//		OUTPUT_PATH = std::string(argv[7]);
//	}
//
//
//
//}