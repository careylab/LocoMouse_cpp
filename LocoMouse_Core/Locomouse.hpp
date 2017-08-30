#ifndef __Locomouse_H_INCLUDED__
#define __Locomouse_H_INCLUDED__
//Forward declare

// Includes
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "MyMat.hpp"
#include "Candidates.hpp"

using namespace cv;

//Defining auxiliary functions:

int LocoMouse_parseInputs(int argc, char* argv[]);

int parseLocoMouseSide(string mouse_side_input);

void firstLastOverT(const Mat& values, uint L, uint32_t first_last[2], uint T);

vector<Candidate> nmsMax(const Mat& detection_box, Size box_size, double overlap);

vector<Candidate> peakClustering(const Mat& detection_box, Size box_size, int cluster_method, double overlap, bool debug);

void readLocoMouse(Mat& I, VideoCapture& V, const Mat& Bkg, Mat& Calibration, const bool flip_image, bool debug);

int loadLocoMouseCalib(const char* file, Mat& calibration, Mat& calibration_inv, Rect& BBtop, Rect& BBbottom);

void correctImage(Mat& Iin, Mat& Iout, Mat& Calibration, bool debug);

void displayPaws(Mat img, vector<Candidate> candidates);

void displayPaws_P22D(Mat img_bottom, Mat img_side, vector<P22D> pair_candidates);

vector<P22D> boxPairingWithVelConstraint(vector<Candidate>& Candidates_b,vector<Candidate>& Candidates_t, const Mat& Ibbb, const Mat& Itbb, const Mat& Ibbb_prev, const Mat& Itbbb_prev, bool vel_check, LocoMouse_Feature& F, int mode, double T, bool debug);

vector<P22D> matchViews(const Mat& boolD, const Mat& D_side_weight, vector<Candidate>& C_b, int N_bottom, vector<Candidate>& C_t, int N_side, bool vel_check, LocoMouse_Feature F, const Mat &Ibbb, const Mat &Itbb, const Mat &Ibbb_prev, const Mat &Itbb_prev, bool debug);

bool checkVelCriterion(const Mat& I, const Mat& I_prev, const Rect & im_box, int box_area, double alpha, double T);

Mat xDist(vector<Candidate> &P1, int N_p1, vector <Candidate> &P2, int N_p2);

MyMat boxWeight(vector<Candidate> &p_candidates, Rect &BB, vector<LocoMouse_LocationPrior> interest_points, bool debug);

MATSPARSE pairwisePotential(vector<Candidate> &Ci, vector<Candidate> &Cip1, Point_<double> &ONG_BR_corner, double grid_spacing, vector< Point_<double> > &ONG, Size ONG_size, double max_displacement_bottom, double alpha_vel, double pairwise_occluded_cost);

MATSPARSE pairwisePotential_side(vector<uint> &Zi, vector<uint> &Zip1, double grid_mapping, double grid_spacing, vector<uint> &ONGi, uint Nong, double max_displacement_bottom, double alpha_vel, double pairwise_occluded_cost);

uint32_t matchToRange(int32_t input, int32_t min_val, int32_t max_val);

Point medianPoint(int* x, int* y, const int N);

void mouseBoundingBox_postProcess(vector<double> &bb_w, vector<double> &bb_hb, vector<double> &bb_ht, int N_frames, Rect &BB_side, Rect &BB_bottom);

double medianvec(vector<double> &v, const int N);

double stdvec(vector<double> &v, int N);

void vecmovingaverage(vector<double> &v, vector<uint32_t> &vout, int N_samples, int N_window);

void computeMouseBox(Mat &I_median, Mat &I, Mat &It, Mat &Ib, uint N_rows, uint N_rows_t, uint N_rows_b, uint N_cols, double &bb_x, double &bb_yb, double &bb_yt, double & bb_w, double & bb_hb, double &bb_ht);

void detectTailBottom(Mat &I_bb_bottom_pad, Rect BB_tail_bottom_pad, Rect BB_tail_bottom_unpad, Mat &Filtered_tail_bottom, LocoMouse_Feature F);

Mat detectTail(Mat &I_bb_bottom_pad, Mat &I_bb_side_pad, Rect BB_tail_bottom_pad, Rect BB_tail_side_pad, Rect BB_tail_bottom_unpad, Rect BB_tail_side_unpad, Mat &Filtered_tail_bottom, Mat &Filtered_tail_side, LocoMouse_Feature F, bool debug);

void imBorderPreservingBB(const Mat& I, Mat& Ibb, Rect BB, uint32_t bb_x, uint32_t bb_y);

void sideViewMatching(Mat& T, vector<Mat> &T_side, vector< vector<P22D> > &candidates_bottom_side_matched, vector<uint> &ONG_side, uint lowest_point, bool debug);

void exportTracks(Mat& M_bottom, const vector<Mat> &M_side, vector< vector<P22D> > &candidates_bottom_side_matched, FileStorage output, Rect BB_bottom, Rect BB_side, vector<uint> bb_x_avg, vector<uint> bb_yb_avg, vector<uint> bb_yt_avg, string feature_name);

void weightedMeanCandidates(vector <Candidate> &Candidates, const Mat& Filtered, LocoMouse_Feature &F, Size pre_padding, bool debug);

bool computeMouseBox_DD(Mat &It, const Mat &H, int N_cols, int N_rows_b, int N_rows_t, double& bb_x, double& bb_yb, double& bb_yt, double& bb_w, double& bb_hb, double& bb_ht, bool debug);

void MATLAB_imadjust_user_parameters(const Mat &Iin, Mat &Iout, double low_in, double high_in, double low_out, double high_out);

void MATLAB_imadjust_default_parameters(const Mat &Iin, Mat &Iout);

void imfill(const Mat &Iin, Mat &Iout, bool debug);

void histogramMatching(Mat &I, Mat& cumsum_ref, bool debug);

int importLocoMouseParameters(string file, string ref_path);/*, int &conn_comp_connectivity, int &median_filter_size, int &min_pixel_visible, double &top_bottom_min_overlap, double &maximum_normalized_beacon_distance, int &max_displacement_bottom, int &max_displacement_side, int &occlusion_grid_spacing_pixels_side, int &occlusion_grid_spacing_pixels, double &occlusion_grid_max_width, double &tail_sub_bounding_box, double &alpha_vel, double &alpha_vel_side, double &pairwise_occluded_cost, int &moving_average_window, int &mode_char, vector<LocoMouse_LocationPrior> &paw_prior, vector<LocoMouse_LocationPrior> &snout_prior, bool &transform_gray_values, Mat &gray_value_transformation);*/

string output_file_name(const string& s);

void exportDebugVariables(uint N_frames, Rect &BB_side, Rect &BB_bottom, vector<uint32_t> bb_x_avg, vector<uint32_t> bb_yb_avg, vector<uint32_t> bb_yt_avg, vector< Point_<double> > ONG, vector<uint> ONG_side, Size ONG_size, vector< vector<P22D> > candidates_paw_bottom_side_matched, vector< vector<P22D> > candidates_snout_bottom_side_matched, const Mat &M_paw_bottom, const vector<Mat> &M_paw_side, const Mat &M_snout_bottom, const vector<Mat> &M_snout_side, vector <MATSPARSE> pairwise_potential_paw_bottom, vector <MATSPARSE> pairwise_potential_snout_bottom, vector <MyMat> unary_potential_paw_bottom, vector <MyMat> unary_potential_snout_bottom);

void selectLargestRegion(const Mat &Iin, Mat & Iout);

#endif
