/*	Defining the LocoMouse problem.
	
	This class configures the tracking problem (loading video, background image, model, calibration, ...) and calls the different routines for computing the feature tracks.


	Author: Joao Fayad (joaofayad@gmail.com)
*/

#ifndef __LocoMouse_Class_H_INCLUDED__
#define __LocoMouse_Class_H_INCLUDED__

//OpenCV dependencies:
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <string.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <numeric>
#include <functional>
#include <map>

//LocoMouse Dependencies:
#include "Candidates.hpp"
#include "MyMat.hpp"
#include "match2nd.h"

class LocoMouse_LocationPrior {
private:
	cv::Point_<double> POSITION;
	double MAX_DISTANCE;
	cv::Rect_<double> AREA;

public:
	LocoMouse_LocationPrior();
	LocoMouse_LocationPrior(double x, double y, double md, double min_x, double max_x, double min_y, double max_y);
	cv::Point_<double> position() const { return POSITION; };
	cv::Rect_<double> area() const { return AREA; };
	double max_distance() const { return MAX_DISTANCE; };
};

class LocoMouse_Parameters {
	//Holds the set of (user configurable) parameters of LocoMouse
public:

	//User configurable via config.yml:

	int conn_comp_connectivity = 8; //Connectivity for the Connected components algorithm.
	int median_filter_size = 11; //Must be an odd integer.
	int min_pixel_visible = 0; //Minimum number of visible pixels to consider a valid BB boundary.
	double top_bottom_min_overlap = 0.7; // Boxes must overlap for at least 70% for matches to be considered valid.
	double maximum_normalized_beacon_distance = 0.4; //Candidates with normalized distance lower than this are suppressed.

	//FIXME: These parameters are set in pixels and they depend on the framerate and resolution of the video. For generalized behaviour calibrate with the training videos and ajdust to incoming box size and framerate. 
	int max_displacement = 15; //Maximum displacement in pixels allowed between two frames (There is a velocity component added to this).
	int max_displacement_top = 15;
	int occlusion_grid_spacing_pixels_top = 20;
	int occlusion_grid_spacing_pixels = 20; //Spacing of the occlusion grid in pixels, highly dependent on resolution.

	double body_proportion_bounding_box = 0.75; //The occlusion grid extends for only this percentage of the BB on the right side.
	double tail_sub_bounding_box = 0.6; //The percentage counting from the left of the box where the tail is looked for.
	double alpha_vel = 1E-1; //Relative term for the velocity costs on the match2nd tracking algorithm.
	double alpha_vel_top = 100;
	double pairwise_occluded_cost = 1E-2; //Cost for moving to and between occluded points on the match2nd algorithm.
	int moving_average_window = 5; //Must be odd. 
	
	int mode_int = 0;
	int use_reference_image = 0;
	int user_provided_bb = 0;

	//To read from inputs:
	std::string REF_PATH = "./";
	bool LM_DEBUG = false;
	int LM_FRAME_TO_DEBUG = -1;
	int LM_N_FRAMES_TO_DEBUG = -1;
	bool LM_VISUAL_DEBUG = false;

	//Default to the System:
	const unsigned int N_paws = 4;
	const unsigned int N_snout = 1;
	const unsigned int N_tail_points = 15;

	std::vector <LocoMouse_LocationPrior> PRIOR_PAW, PRIOR_SNOUT;
	
	//Matrix with the permutations for the paws:
	const unsigned int N_PAW_PERMUTATIONS = 24;
	const cv::Mat PAW_PERMUTATIONS = (cv::Mat_<int>(4, N_PAW_PERMUTATIONS) << 3, 2, 1, 0, 3, 2, 0, 1, 3, 1, 2, 0, 3, 1, 0, 2, 3, 0, 1, 2, 3, 0, 2, 1, 2, 3, 1, 0, 2, 3, 0, 1, 2, 1, 3, 0, 2, 1, 0, 3, 2, 0, 1, 3, 2, 0, 3, 1, 1, 2, 3, 0, 1, 2, 0, 3, 1, 3, 2, 0, 1, 3, 0, 2, 1, 0, 3, 2, 1, 0, 2, 3, 0, 2, 1, 3, 0, 2, 3, 1, 0, 1, 2, 3, 0, 1, 3, 2, 0, 3, 1, 2, 0, 3, 2, 1);
	cv::Mat REF_CDF = cv::Mat::zeros(1, 256, CV_32FC1); //Matrix used to store the normalized CDF from a reference image.

	//Constructors:
	LocoMouse_Parameters();
	LocoMouse_Parameters(std::string config_file_name);

	//Assignment:
	LocoMouse_Parameters& operator=(LocoMouse_Parameters &&P); //move assignment?
	//LocoMouse_Parameters& operator=(const LocoMouse_Parameters &P); //copy assignment?


	//Destructor:
	//~LocoMouse_Parameters();

};

//--------- LocoMouse_Feature
class LocoMouse_Feature {
private:

	cv::Mat w_b, w_s; //Detector matrices
	cv::Mat w_avg_i, w_avg_j; //Weighted average indices for nms
	double rho_b, rho_s; //Detector bias
	cv::Size size_b, size_s; //Size of the detectors
	cv::Rect match_rect_b, match_rect_s; //Overlap mask for velocity matching

public:

	LocoMouse_Feature();
	LocoMouse_Feature(cv::Mat window_bottom, cv::Mat window_side, double bias_bottom, double bias_side);
	LocoMouse_Feature& operator=(LocoMouse_Feature& other);

	//Models:
	cv::Mat detector_bottom() const;
	cv::Mat detector_side() const;

	//Biases:
	double rho_bottom() const;
	double rho_side() const;

	//Sizes:
	cv::Size size_bottom() const;
	cv::Size size_side() const;

	//Weighted Average:
	cv::Mat weighted_average_i() const;
	cv::Mat weighted_average_j() const;

	//Velocity Matching windows:
	cv::Rect match_box_bottom() const;
	cv::Rect match_box_side() const;

};


class LocoMouse_Model {

public:
	LocoMouse_Feature paw, snout, tail;
	cv::Size spre_b, spre_t, spost_b, spost_t;

	LocoMouse_Model(); //Default constructor.
	~LocoMouse_Model(); //Default destructor.
	LocoMouse_Model(std::string file_name);
	LocoMouse_Model& operator=(LocoMouse_Model &other);

	//Appending sizes:
	cv::Size size_pre_bottom() const;
	cv::Size size_pre_side() const;

	cv::Size size_post_bottom() const;
	cv::Size size_post_side() const;

};

class LocoMouse {

protected:

	LocoMouse_Parameters LM_PARAMS;

	//Input Processing:
	std::string LM_CALL, CONFIG_FILE, VIDEO_FILE, BKG_FILE, MODEL_FILE, CALIBRATION_FILE, FLIP_CHAR, OUTPUT_PATH;

	std::string ref_path, output_file, debug_file, debug_text;
	cv::FileStorage OUTPUT, DEBUG_OUTPUT;
	std::ofstream DEBUG_TEXT;
	
	//Image and Video processing related parameters:
	cv::VideoCapture V;
	cv::Mat I_PAD, I, I_PREV_PAD, BKG;
	cv::Mat CALIBRATION, CALIBRATION_INV;
	cv::Mat I_BOTTOM_MOUSE_PAD, I_BOTTOM_MOUSE, I_SIDE_MOUSE_PAD, I_SIDE_MOUSE, I_BOTTOM_MOUSE_PAD_PREV, I_SIDE_MOUSE_PAD_PREV;
	cv::Mat DETECTION_SCORES_PAW_BOTTOM, DETECTION_SCORES_PAW_SIDE, DETECTION_SCORES_SNOUT_BOTTOM, DETECTION_SOCRES_SNOUT_SIDE;
	cv::Mat TAIL_MASK;

	bool IMAGE_FLIP;

	//Bounding boxes:
	cv::Rect I_UNPAD;
	cv::Rect BB_SIDE_VIEW, BB_BOTTOM_VIEW;
	cv::Rect BB_BOTTOM_MOUSE, BB_SIDE_MOUSE, BB_BOTTOM_MOUSE_PAD, BB_SIDE_MOUSE_PAD;
	cv::Rect BB_UNPAD_MOUSE_BOTTOM, BB_UNPAD_MOUSE_SIDE;

	cv::Rect BB_BOTTOM_TAIL_PAD, BB_UNPAD_TAIL_BOTTOM, BB_BOTTOM_TAIL, BB_SIDE_TAIL_PAD, BB_UNPAD_TAIL_SIDE, BB_SIDE_TAIL;
	std::vector< unsigned int> BB_X_POS, BB_Y_SIDE_POS, BB_Y_BOTTOM_POS;

	//Image size, Video size, Padding sizes:
	unsigned int N_FRAMES;
	unsigned int N_ROWS, N_COLS;
	int CURRENT_FRAME = -1, PAD_PRE_ROWS, PAD_PRE_COLS, PAD_POST_ROWS, PAD_POST_COLS; //For implementation reasons they are set to int.
	
	//Occlusion Grid Parameters;
	cv::Point_<double> ONG_BR_corner;
	std::vector< Point_<double> > ONG;
	cv::Size ONG_size;
	
	//Occlusion Grid Side View:
	vector<unsigned int> ONG_SIDE;
	unsigned int ONG_SIDE_LOWEST_POINT;
	
	//Model:
	LocoMouse_Model M;
	
	//Candidates:
	vector < vector <Candidate> > CANDIDATES_BOTTOM_PAW;
	vector < vector <Candidate> > CANDIDATES_BOTTOM_SNOUT;

	vector < vector <Candidate> > CANDIDATES_SIDE_PAW;
	vector < vector <Candidate> > CANDIDATES_SIDE_SNOUT;
	
	//P22D Candidates:
	vector < vector <P22D> > CANDIDATES_MATCHED_VIEWS_PAW;
	vector < vector <P22D> > CANDIDATES_MATCHED_VIEWS_SNOUT;

	std::vector <MyMat> UNARY_BOTTOM_PAW, UNARY_BOTTOM_SNOUT;

	std::vector <MATSPARSE> PAIRWISE_BOTTOM_PAW, PAIRWISE_BOTTOM_SNOUT;

	//Tracking:
	cv::Mat TRACK_INDEX_PAW_BOTTOM, TRACK_INDEX_SNOUT_BOTTOM, TRACK_INDEX_PAW_SIDE, TRACK_INDEX_SNOUT_SIDE;
	
	vector<cv::Mat> TRACKS_TAIL;

	//Private methods
	void loadVideo();

	void loadBackground();

	void loadCalibration();

	void loadFlip();

	void validateImageVideoSize();

	void configurePath();

	void correctImage(cv::Mat& Iin, cv::Mat& Iout);

	void selectLargestRegion(const Mat &Iin, Mat &Iout);

	vector <Candidate> detectPointCandidatesBottom(cv::Mat& I_VIEW_PAD, cv::Rect BB_UNPAD, LocoMouse_Feature M_FEAT, cv::Mat &I_bb_bottom_mask);

	vector <Candidate> detectPointCandidatesSide(cv::Mat& I_VIEW_PAD, cv::Rect BB_UNPAD, LocoMouse_Feature M_FEAT, cv::Mat &I_bb_bottom_mask);

	void computeMouseBox(cv::Mat &I_median, cv::Mat &I, cv::Mat &I_side_view, cv::Mat &I_bottom_view, double& bb_x, double& bb_y_bottom, double& bb_y_side, double& bb_width, double& bb_height_bottom, double& bb_height_side, const LocoMouse_Parameters &LM_PARAMS);

	MATSPARSE pairwisePotential(vector<Candidate> &Ci, vector<Candidate> &Cip1, Point_<double> &grid_mapping, double grid_spacing, vector< Point_<double> > &ONGi, Size ONG_size, double max_displacement, double alpha_vel, double pairwise_occluded_cost);

	MATSPARSE pairwisePotential_SideView(const vector<uint> &Zi, const vector<uint> &Zip1, double grid_mapping, double grid_spacing, const vector<uint> &ONGi, const unsigned int Nong, const double max_displacement, const double alpha_vel, const double pairwise_occluded_cost);

	MyMat unaryCostBox(vector<Candidate> &p_candidates, cv::Rect &BB, vector<LocoMouse_LocationPrior> location_prior);

	bool checkVelCriterion(const cv::Mat& I, const cv::Mat& I_prev, const cv::Rect& im_box, int box_area, double alpha, double T);

	void exportTracksBottom(const cv::Mat& T_bottom, const vector< vector <Candidate> > &candidates_bottom, const string feature_name, const unsigned int N_features);

	void exportPointTracks(const cv::Mat& T_bottom, const cv::Mat &T_top, const vector< vector<P22D> > &candidates_bottom_top_matched, const string feature_name, const unsigned int N_features);

	void LocoMouse::exportLineTracks(std::vector <cv::Mat> Tracks, std::string track_name, int N_line_points);

	vector<P22D> matchViews(const cv::Mat &boolD, const cv::Mat &D_top_weight, const vector<Candidate> &C_b, const vector<Candidate> &C_t, bool vel_check, LocoMouse_Feature F, const cv::Mat &Ibbb, const cv::Mat &Itbb, const cv::Mat &Ibbb_prev, const cv::Mat &Itbb_prev, const Point_<int> padding_pre_bottom, const Point_<int> padding_pre_side, bool debug);

	cv::Mat xDist(const vector<Candidate> &P1, const vector <Candidate> &P2);

	vector<P22D> matchingWithVelocityConstraint(vector<Candidate>& Candidates_b, vector<Candidate>& Candidates_t, const cv::Mat& Ibbb, const cv::Mat& Itbb, const cv::Mat& Ibbb_prev, const cv::Mat& Itbbb_prev, const Point_<int> padding_pre_bottom, const Point_<int> padding_pre_side, bool vel_check, LocoMouse_Feature& F, double T, bool debug);

	void computeMouseBoxSize(std::vector<double> &bb_w, std::vector<double> &bb_hb, std::vector<double> &bb_ht, cv::Rect &BB_top, cv::Rect &BB_bottom);

	void cropImage(cv::Mat &Iin, cv::Mat &Iout, cv::Rect BB);

	cv::Mat LocoMouse::bestSideViewMatch(const cv::Mat& T, const vector< vector<P22D> > &candidates_bottom_top_matched, const vector<uint> &ONG_top, const unsigned int lowest_point, const unsigned int N_features);

	cv::Mat LocoMouse::detectLineCandidates(const LocoMouse_Feature F, const unsigned int N_line_points, cv::Mat & Mask);

	void exportDebugVariables();

public:
	
	
	//Constructors:
	LocoMouse(int argc, char* argv[]);
		
	//Destructor
	//~LocoMouse();
	
	//Methods:
	void readFrame(); //Reads and processes next frame into I
	
	void readFrame(cv::Mat& I); //Reads and processes next frame into provided matrix.

	void computeBoundingBox(); //Computes mouse position and bounding box along the video
	
	void initializeFeatureLoop(); //Initializes Feature extraction loop based on computed bounding boxes.

	void cropBoundingBox(); //Crops the mouse on the bottom and side view.

	void detectTail(); //Searches for the tail.
	
	void detectBottomCandidates(); //Detects the candidates for the Paws and Snout on the bottom view.

	void computeUnaryCostsBottom(); //Computes unary costs for the bottom view.

	void computePairwiseCostsBottom(); //Computes pairwise costs for the bottom view.
	
	void detectSideCandidates(); //Detects candidates for the PAws and Snout on the side view.
	
	void matchBottomSideCandidates(); //Matches Side view candidates to Bottom view candidates.

	void storePreviousImage(); //Copies the current frame to be used on the next step;

	void computeBottomTracks(); //Computes the final trajectories for the bottom view.

	void computeSideTracks(); //Computes the best bottom view tracks based on the side view tracks.

	void computeTracks(); //Computes the final trajectories for the bottom and side views.

	void exportFeatureTracks();

	void exportResults();

	void imadjust(const cv::Mat &Iin, cv::Mat &Iout, double low_in, double high_in, double low_out, double high_out);

	void imadjust_default(const cv::Mat &Iin, cv::Mat &Iout);

	//Getters:
	unsigned int N_frames() const;
};

double medianvec(std::vector<double> &v, const int N);

double stdvec(std::vector<double> &v, int N);

void vecmovingaverage(std::vector<double> &v, std::vector<unsigned int> &vout, int N_window);

void histogramMatching(cv::Mat &Iin, const cv::Mat &reference_cdf);

vector<Candidate> nmsMax(const cv::Mat& detection_box, Size box_size, double overlap);

vector<Candidate> peakClustering(const cv::Mat& detection_box, Size box_size, int cluster_method, double overlap, bool debug);

template <typename T>
T matchToRange(T input, T min_val, T max_val);

void computeNormalizedCDF(const cv::Mat &Iin, cv::Mat &CDF);

//FIXME: Example copied from http://stackoverflow.com/questions/12261915/howto-throw-stdexceptions-with-variable-messages to better handle exception messages.
class Stream_Formatter
{
public:
	Stream_Formatter() {}
	~Stream_Formatter() {}

	template <typename Type>
	Stream_Formatter & operator << (const Type & value)
	{
		stream_ << value;
		return *this;
	}

	std::string str() const { return stream_.str(); }
	operator std::string() const { return stream_.str(); }

	enum ConvertToString
	{
		to_str
	};
	std::string operator >> (ConvertToString) { return stream_.str(); }

private:
	std::stringstream stream_;

	Stream_Formatter(const Stream_Formatter &);
	Stream_Formatter & operator = (Stream_Formatter &);
};

//Auxiliary functions:
template <typename T>
void firstLastOverT(const cv::Mat& values, const unsigned int L, T first_last[2], const T th);


#endif

