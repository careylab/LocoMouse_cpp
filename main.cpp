/* LocoMouse C++ Version

	Written by Joao Fayad (joaofayad@gmail.com)

*/

//OpenCV includes:
//#include <opencv2/core.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/highgui.hpp>

#include <opencv2/core.hpp>

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
#include <memory>


//LocoMouse includes:
#include "MyMat.hpp"
#include "Candidates.hpp"
#include "match2nd.h"

//FIXME: New classes merge with other classes
#include "LocoMouse_class.hpp"
#include "LocoMouse_Methods.hpp"

int main(int argc, char* argv[]) {
	double t = (double) getTickCount();
	int return_val = EXIT_SUCCESS;

	try {

		//Parsing the inputs:
		LocoMouse_ParseInputs inputs = LocoMouse_ParseInputs(argc, argv);

		//Initialize LocoMouse problem
		std::unique_ptr<LocoMouse> L = LocoMouse_Initialize(inputs);

		//Compute Bounding Box location accross video:
		L->getBoundingBox();

		L->initializeFeatureLoop();

		//Compute per-frame candidate locations:
		for (unsigned int i_frames = 0; i_frames < L->N_frames(); ++i_frames) {
			
			//Read new image:
			L->readFrame();

			//Crop image bounding boxes for both views:
			L->cropBoundingBox();

			// --- Detecting the Tail:
			L->detectTail();
			
			// --- Detecting Paw + Snout candidates for bottom view:
			L->detectBottomCandidates();
			
			// --- Computing Unary costs based on Location Prior:
			L->computeUnaryCostsBottom();

			// --- Computing Pairwise costs (Displacement between frames):
			L->computePairwiseCostsBottom();

			// --- Detecting Paw + Snout candidates for side view:
			L->detectSideCandidates();
			
			// --- Matching 
			L->matchBottomSideCandidates();

			L->storePreviousImage();

		}

		//--- Paws + Snout tracks:
		L->computeBottomTracks();

		//Estimate corresponding Z coordinates:
		L->computeSideTracks();

		//--- Export results:
		L->exportResults();

	}
	catch (std::invalid_argument e) {
		std::cout << "Invalid inputs: " << e.what() << std::endl;
		return_val = EXIT_FAILURE;
	}
	catch (std::runtime_error e) {
		std::cout << "Runtime Error: " << e.what() << std::endl;
		return_val = EXIT_FAILURE;
	}

	t = ((double)getTickCount() - t) / getTickFrequency();
	std::cout << "Total Elapsed time: " << t << "s" << std::endl;
	return return_val;
}