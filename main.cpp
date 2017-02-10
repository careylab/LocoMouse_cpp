/* LocoMouse C++ Version

	Written by Joao Fayad (joaofayad@gmail.com)

*/

//OpenCV includes:
#include <opencv2/core.hpp>
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

std::ofstream debug_text;
cv::FileStorage debug_locomouse;

int main(int argc, char* argv[]) {
	double t = (double)getTickCount();

	try {

		//FIXME: Implement this properly after debug:
		/*debug_text.open("debug.txt");
		debug_locomouse.open("debug_locomouse.yml", cv::FileStorage::WRITE);
		debug_text << "Start debug" << std::endl;*/

		//Initialize LocoMouse problem
		LocoMouse L(argc, argv);

		//FIXME: Figure out a way of throwing exceptions if the initialization fails!
		//Compute Bounding Box location accross video:
		L.computeBoundingBox();

		L.initializeFeatureLoop();

		//Compute per-frame candidate locations:
		for (unsigned int i_frames = 0; i_frames < L.N_frames(); ++i_frames) {
			//debug_text << "Current frame: " << i_frames << std::endl;
			
			//Read new image:
			L.CURRENT_FRAME = i_frames;
			L.readFrame();
			//debug_text << "Read Frame" << std::endl;

			//Crop image bounding boxes for both views:
			L.cropBoundingBox();
			//debug_text << "Cropped Box" << std::endl;

			// --- Detecting the Tail:
			L.detectTail();
			//debug_text << "Bottom Candidates" << std::endl;

			// --- Detecting Paw + Snout candidates for bottom view:
			L.detectBottomCandidates();
			//debug_text << "Bottom Candidates" << std::endl;

			// --- Computing Unary costs based on Location Prior:
			L.computeUnaryCostsBottom();
			//debug_text << "Unary Cost" << std::endl;

			// --- Computing Pairwise costs (Displacement between frames):
			if (i_frames > 0) {
				L.computePairwiseCostsBottom();
				//debug_text << "Pairwise Cost" << std::endl;
			}

			// --- Detecting Paw + Snout candidates for side view:
			L.detectSideCandidates();
			//debug_text << "Detect Side Candidates" << std::endl;
			
			// --- Matching 
			L.matchBottomSideCandidates();
			//debug_text << "Match Bottom and Side Views" << std::endl;
			
			L.I_PAD.copyTo(L.I_PREV_PAD);

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
	catch (std::exception& e) {
		std::cout << e.what() << endl;
	}

	debug_text.close();
	debug_locomouse.release();

	t = ((double)getTickCount() - t) / getTickFrequency();
	cout << "Total Elapsed time: " << t << "s" << endl;
	return EXIT_SUCCESS;
}
