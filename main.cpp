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

int main(int argc, char* argv[]) {
	double t = (double) getTickCount();

	try {

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
		
			//Crop image bounding boxes for both views:
			L.cropBoundingBox();

			// --- Detecting the Tail:
			L.detectTail();
			
			// --- Detecting Paw + Snout candidates for bottom view:
			L.detectBottomCandidates();
			
			// --- Computing Unary costs based on Location Prior:
			L.computeUnaryCostsBottom();

			// --- Computing Pairwise costs (Displacement between frames):
			L.computePairwiseCostsBottom();

			// --- Detecting Paw + Snout candidates for side view:
			L.detectSideCandidates();
			
			// --- Matching 
			L.matchBottomSideCandidates();

			L.storePreviousImage();

		}

		//--- Paws + Snout tracks:
		L.computeBottomTracks();

		//Estimate corresponding Z coordinates:
		L.computeSideTracks();

		//--- Export results:
		L.exportResults();

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