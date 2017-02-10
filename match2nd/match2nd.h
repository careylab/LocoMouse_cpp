//match2nd.h
#ifndef __match2nd_H_INCLUDED__
#define __match2nd_H_INCLUDED__

//Forawrd declare

//Include
#include <opencv2/core.hpp>
#include <algorithm>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <cstring>
#include <vector>
#include "MyMat.hpp"
#include <iostream>

using namespace std;
using namespace cv;

//Auxiliary code for match2nd:
extern vector<int> locations;
extern int occ;
extern double occ_score;
extern double BAM;
extern ofstream mylog;

class point {
public:
	double** message;
	const double** unary;

	inline double un(unsigned int a, unsigned int b, unsigned int not_occ, double occ_cost_i) {
		//NOTE: Assumes unary occlusion cost to be fixed for all ocluded points.
		assert(a < locations.size());
		//assert(b < (locations[a])); //CHANGE
		assert(b < (locations[a] + occ));
		if (b < not_occ) {
			return unary[a][b] + message[a][b];
		}
		else {
			return (occ_cost_i + message[a][b]);
		}
	}
	inline void messupdate(unsigned int a, unsigned int b, double c) {
		assert(a < locations.size());
		//assert(b < (locations[a]+occ)); //CHANGE
		assert(b < (locations[a] + occ));
		message[a][b] += c;
	}
	inline double& mback(unsigned int a, unsigned int b) {
		assert(a < locations.size());
		assert(b < transitions[a]);
		return(marginbackward[a][b]);
	}
	inline double& mfore(unsigned int a, unsigned int b) {
		assert(a < locations.size());
		assert(b < transitions[a]);
		return(marginforward[a][b]);
	}
	inline double pa(unsigned int a, unsigned int b) {
		assert(a < locations.size());
		assert(b < transitions[a]);
		return(pairwise[a][b]);
	}
	inline int pae(unsigned int a, unsigned int b) {
		assert(a < locations.size());
		assert(b < transitions[a]);
		return(pairend[a][b]);
	}
	inline int pas(unsigned int a, unsigned int b) {
		assert(a < locations.size());
		//assert(b < locations[a] + 1); //CHANGE
		assert(b < (locations[a] + occ + 1));
		return(pairstart[a][b]);
	}

	vector < double* > marginbackward;
	vector < double* > marginforward;

	vector <const double*> pairwise;
	vector <const int*> pairend;
	vector <const int* > pairstart;
	vector <unsigned int > transitions;
	vector < int > labelling;
	vector < double > best;
	vector < int > bestlocation;
	vector < double > secondbest;

	point() {

		marginforward.resize(locations.size() - 1);
		marginbackward.resize(locations.size() - 1);

		labelling.resize(locations.size());
		best.resize(locations.size());
		secondbest.resize(locations.size());
		bestlocation.resize(locations.size());

		pairwise.resize(locations.size());
		pairend.resize(locations.size());
		pairstart.resize(locations.size());
		transitions.resize(locations.size());

	}

	void inspect() {
		//mylog<<"INSPECT"<<endl;
		// //mylog<<"-------"<<endl;
		// //mylog<<"Unarys"<<endl;
		// for (unsigned int i = 0; i != locations.size() ; ++i){
		//   //mylog<<"i: "<<i<< " unary[i] : "<<(unsigned long)(unary[i])<<endl;
		//   for (unsigned int j = 0; j != locations[i] ; ++j){
		// 	//mylog<<un(i,j)<<' ';
		//   }
		//   //mylog<<endl;
		// }

		// //mylog<<"Marginforwards"<<endl;
		// //mylog<<"marginfoward size:"<<marginforward.size()<<endl;

		// for (unsigned int i = 0; i != locations.size()-1 ; ++i){
		//   //mylog<<"marginforward[i] : "<<(unsigned long)(marginforward[i])<<endl;
		//   for (unsigned int j = 0; j != transitions[i] ; ++j){
		// 	//mylog<<mfore(i,j)<<' ';
		//   }
		//   //mylog<<endl;
		// }
		// //mylog<<"Marginbackwards"<<endl;
		// for (unsigned int i = 0; i != locations.size()-1 ; ++i){
		//   //mylog<<"marginbackward[i] : "<<(unsigned long)(marginbackward[i])<<endl;
		//   for (unsigned int j = 0; j != transitions[i] ; ++j){
		// 	//mylog<<mback(i,j)<<' ';
		//   }
		//   //mylog<<endl;
		// }
		//mylog<<"Labelling"<<endl;
		for (unsigned int i = 0; i != locations.size(); ++i) {
			//mylog<<(unsigned long)(labelling[i])<<' ';
		}
		//mylog<<endl;

		//mylog<<"Best"<<endl;
		for (unsigned int i = 0; i != locations.size(); ++i) {
			//mylog<<(best[i])<<' ';
		}
		//mylog<<endl;

		//mylog<<"Secondbest"<<endl;
		for (unsigned int i = 0; i != locations.size(); ++i) {
			//mylog<<(secondbest[i])<<' ';
		}
		//mylog<<endl;

		//mylog<<"bestlocation"<<endl;
		for (unsigned int i = 0; i != locations.size(); ++i) {
			//mylog<<(bestlocation[i])<<' ';
		}
		//mylog<<endl;
	}

	void clearmargin() {
		for (unsigned int frame = 0; frame != locations.size(); ++frame)
			labelling[frame] = -2;
		for (unsigned int frame = 0; frame != locations.size() - 1; ++frame)
			for (unsigned int i = 0; i != transitions[frame]; ++i)
				mback(frame, i) = -INFINITY;
		for (unsigned int frame = 0; frame != locations.size() - 1; ++frame)
			for (unsigned int i = 0; i != transitions[frame]; ++i)
				mfore(frame, i) = -INFINITY;
	}

	void update_unary_first(unsigned int frame) {
		//mylog<<"update_first"<<endl;
		// This handles the matching term
		//if (bestlocation[frame] < locations[frame] - occ) {//CHANGE
		if (bestlocation[frame] < locations[frame]) {
			//mylog<<"frame "<<frame<<" penalising location "<<bestlocation[frame]<<" by "<<secondbest[frame]-best[frame]<<endl;
			messupdate(frame, bestlocation[frame], secondbest[frame] - best[frame] - BAM);
		}
	}
	void forward_point(unsigned int frame) {
		//mylog<<"Forward_point frame: "<<frame<<endl;
		//mylog<<"locations.size(): "<<locations.size()-1<<endl;
		assert(frame < locations.size() - 1);

		if (frame) {       //BP
			for (unsigned int jj = 0, tick = 0; jj != transitions[frame - 1]; ++jj) {
				//mylog<<"jj: "<<jj<<endl;
				unsigned int j = pae(frame - 1, jj);
				for (unsigned int kk = pas(frame, j); kk != pas(frame, j + 1); ++kk, ++tick) {
					//mylog<<"kk : "<<(unsigned long)(kk)<<endl;  
					//mylog<<"mfore(frame-1,jj): "<<mfore(frame-1,jj)<<endl;
					////mylog<<"tertiary : "<<(unsigned long)(tertiary)<<endl;
					////mylog<<"tertiary[frame-1] : "<<(unsigned long)(tertiary[frame-1])<<endl;
					////mylog<<"tertiary[frame-1][tick]: "<<tertiary[frame-1][tick]<<endl;
					double cost = mfore(frame - 1, jj); //FIXME:+tertiary[frame-1][tick];
														//mylog<<"cost: "<<cost<<endl;
					mfore(frame, kk) = max(mfore(frame, kk), cost);
				}
			}
		}
		else {
			for (unsigned int i = 0; i != (locations[frame] + occ); ++i) { //for (unsigned int i = 0; i != locations[frame]; ++i) { //CHANGE
				 //unsigned int not_occ = locations[frame] - occ;//CHANGE
				//mylog<<"pas(frame,i): "<<pas(frame,i)<<endl;
				//mylog<<"pas(frame,i+1): "<<pas(frame,i+1)<<endl;
				for (unsigned int jj = pas(frame, i); jj != pas(frame, i + 1); ++jj) {
					mfore(frame, jj) = un(frame, i, locations[frame], occ_score);
					/*if (i > 0 || i < 10) {
						cout << "un(" << frame << "," << i << "): " << mfore(frame, jj) << " " << locations[frame] << " " << BAM << endl;
					}*/
					//mylog<<"i: "<<i<<" jj: "<<jj<<" cost "<<un(frame,i)<<' '<<mfore(frame,jj) <<endl;
				}
			}
		}
		for (unsigned int i = 0; i != transitions[frame]; ++i) {
			//mfore(frame, i) += un(frame + 1, pae(frame, i), locations[frame + 1] - occ, BAM) + pa(frame, i); //CHANGE
			mfore(frame, i) += un(frame + 1, pae(frame, i), locations[frame + 1], occ_score) + pa(frame, i);
		}
	}

	void backward_point(unsigned int frame) {
		//mylog<<"BACKWARDS POINT"<<endl;
		//mylog<<"frame: "<<frame<<endl;
		//mylog<<"locations.size(): "<<locations.size()<<endl;
		assert(frame < locations.size() - 1);
		//BP
		if (frame < locations.size() - 2)
			for (unsigned int jj = 0, tick = 0; jj != transitions[frame]; ++jj) {
				unsigned int j = pae(frame, jj);
				for (unsigned int kk = pas(frame + 1, j); kk != pas(frame + 1, j + 1); ++kk, ++tick) {
					//mylog<<"mback(frame,jj): "<<mback(frame,jj)<<endl;
					////mylog<<"tertiary[frame][tick]: "<<tertiary[frame][tick]<<endl;
					double cost = mback(frame + 1, kk);//FIXME:+tertiary[frame][tick];
													   //mylog<<"cost: "<<cost<<endl;
					mback(frame, jj) = max(mback(frame, jj), cost);
					//mylog<<"line 230: frame: "<<frame<<" jj: "<<jj<<" mback(frame,jj): "<<mback(frame,jj)<<endl;
				}
			}
		else {
			for (unsigned int jj = 0; jj != transitions[frame]; ++jj) {
				//mback(frame, jj) = un(frame + 1, pae(frame, jj), locations[frame + 1] - occ, BAM); //CHANGE
				mback(frame, jj) = un(frame + 1, pae(frame, jj), locations[frame + 1], occ_score);
				//mylog<<"frame: "<<frame<<" jj: "<<jj<<" mback(frame,jj): "<<mback(frame,jj)<<endl;
			}

		}
		for (unsigned int i = 0; i != (locations[frame] + occ); ++i) { //for (unsigned int i = 0; i != locations[frame]; ++i) { //CHANGE
			for (unsigned int jj = pas(frame, i); jj != pas(frame, i + 1); ++jj) {
				//mylog<<"jj: "<<jj;
				//mback(frame, jj) += pa(frame, jj) + un(frame, i, locations[frame] - occ, BAM); //CHANGE
				mback(frame, jj) += pa(frame, jj) + un(frame, i, locations[frame], occ_score);
				//mylog<<" mback(frame,jj): "<<mback(frame,jj)<<endl;	
			}
		}
	}


	void findbest(unsigned int frame) {
		//mylog<<"findbest "<<frame<<endl;
		//mylog<<"---------"<<endl;
		assert(frame < locations.size() - 1);
		//call before unary potentials
		best[frame] = -INFINITY;
		best[frame + 1] = -INFINITY;
		secondbest[frame] = -INFINITY;
		secondbest[frame + 1] = -INFINITY;

		unsigned int loc = 0;
		for (unsigned int i = 0; i != transitions[frame]; ++i) {
			while (pas(frame, loc + 1) <= i)++loc;
			//mylog<<"loc: "<<loc<<endl;
			//mylog<<"pae(frame,i): "<<pae(frame,i)<<endl;
			//mylog<<"i: "<<i<<endl;
			//mylog<<"mback(frame,i): "<<mback(frame,i)<<endl;
			//mylog<<"mfore(frame,i): "<<mfore(frame,i)<<endl;
			//mylog<<"un(frame,loc) : "<<(un(frame,loc))<<endl;
			//double temp = mback(frame, i) + mfore(frame, i) - un(frame, loc, locations[frame] - occ, BAM) - un(frame + 1, pae(frame, i), locations[frame + 1] - occ, BAM) - pa(frame, i); //CHANGE
			double temp = mback(frame, i) + mfore(frame, i) - un(frame, loc, locations[frame], occ_score) - un(frame + 1, pae(frame, i), locations[frame + 1], occ_score) - pa(frame, i);
			//mylog<<"temp: "<<temp<<endl;


			if (best[frame] < temp) {
				if (bestlocation[frame] == loc)// only change best
					best[frame] = temp;
				else {
					secondbest[frame] = best[frame];
					best[frame] = temp;
					bestlocation[frame] = loc;
				}
				if (bestlocation[frame + 1] == pae(frame, i))// only change best
					best[frame + 1] = temp;
				else {
					secondbest[frame + 1] = best[frame + 1];
					best[frame + 1] = temp;
					bestlocation[frame + 1] = pae(frame, i);
				}
			}
			else {
				if ((secondbest[frame] < temp) && (bestlocation[frame] != loc))
					secondbest[frame] = temp;
				if ((secondbest[frame + 1] < temp) && (bestlocation[frame + 1] != pae(frame, i)))
					secondbest[frame + 1] = temp;
			}

		}
		//mylog<<"best[frame]: "<<best[frame]<<endl;
		//mylog<<"secondbest[frame]: "<<secondbest[frame]<<endl;
		//mylog<<"secondbest[frame+1]: "<<secondbest[frame+1]<<endl;
		//mylog<<"bestlocation[frame] : "<<(unsigned long)(bestlocation[frame])<<endl;
		//mylog<<"bestlocation[frame+1]: "<<bestlocation[frame+1]<<endl;

		//mylog<<"Done findbest."<<endl;
	}

	void forward_set(unsigned int frame) {
		//mylog<<"forward_set frame: "<<frame<<endl;
		assert(frame < locations.size());
		//BP
		double best = -INFINITY;

		if (frame == 1)
			return;
		if (!frame) {
			for (unsigned int loc = 0; loc != (locations[frame] + occ); ++loc) { //for (unsigned int loc = 0; loc != locations[frame]; ++loc){ //CHANGE
				for (unsigned int i = pas(frame, loc); i != pas(frame, loc + 1); ++i) {
					double cost = mback(frame, i);
					//mylog<<"cost: "<<cost<<" loc: "<<loc<<" pae(frame,i): "<<pae(frame,i)<<endl;

					if (best <= cost) {
						best = cost;
						//mylog<<"best: "<<best<<endl;
						labelling[frame] = loc;
						//mylog<<"labelling[frame]: "<<labelling[frame]<<endl;
						labelling[frame + 1] = pae(frame, i);
						//mylog<<"labelling[frame+1]: "<<labelling[frame+1]<<endl;
					}
				}
			}
			if (best == -INFINITY) {
				//mylog<<"Warning no satisfyable labellings"<<endl;
				labelling[frame] = -1;
				labelling[frame + 1] = -1;
			}
			return;
		}

		for (unsigned int i = 0, tick = 0, loc = 0; i != transitions[frame - 2]; ++i) {
			while (pas(frame - 2, loc + 1) <= i)++loc;
			//mylog<<"loc: "<<loc<<endl;
			unsigned int j = pae(frame - 2, i);
			//mylog<<"j: "<<j<<endl;
			//mylog<<"tick: "<<tick<<endl;
			if ((loc != labelling[frame - 2]) || (j != labelling[frame - 1]))
				tick += pas(frame - 1, j + 1) - pas(frame - 1, j);
			else
				for (unsigned int k = pas(frame - 1, j); k != pas(frame - 1, j + 1); ++k, ++tick) {
					double cost = pa(frame - 1, k) + mback(frame - 1, k);//FIXME:+tertiary[frame-2][tick]
					if (best < cost) {
						best = cost;
						//mylog<<"best: "<<best<<endl;
						labelling[frame] = pae(frame - 1, k);
						//mylog<<"labelling[frame]: "<<labelling[frame]<<endl;
					}
				}
		}
		if (best == -INFINITY)
			labelling[frame] = -1;
	}
	void update_unary_second_pre(int frame) {
		if (BAM == INFINITY)
			message[frame][bestlocation[frame]] = 0;
		else
			if (bestlocation[frame] < locations[frame]) //(bestlocation[frame] < locations[frame] - occ) //CHANGE
				messupdate(frame, bestlocation[frame], -secondbest[frame] + best[frame] + BAM);
	}

	void update_unary_second(int frame) {
		if ((labelling[frame] < locations[frame]) && (labelling[frame] >= 0)) //((labelling[frame] < locations[frame] - occ) && (labelling[frame] >= 0)) //CHANGE
		{
			//mylog<<"prohibiting location "<<labelling[frame]<<endl;
			messupdate(frame, labelling[frame], -INFINITY);
		}
	}

	void margin() {
		//mylog<<"clearmargin"<<endl;
		clearmargin(); //cout << "clearmargin ok" << endl;
		for (unsigned int i = 0; i != locations.size() - 1; ++i) {
			//mylog<<"forwardpoint i: "<<i<<endl;
			forward_point(i);
			//cout << "forward point " << i << " ok!" << endl;
		}
		for (int i = locations.size() - 2; i != -1; --i) {
			//mylog<<"backward point i: "<<i<<endl;
			backward_point(i);
		}
		for (unsigned int i = 0; i != locations.size() - 1; ++i) {
			//mylog<<"find best i: "<<i<<endl;
			findbest(i);
		}
		//mylog<<"update unaries first"<<endl;
		for (unsigned int i = 0; i != locations.size(); ++i)
			update_unary_first(i);
		inspect();
	};
	void disp_margin() {
		//mylog<<"backwardmargin:"<<endl;
		for (unsigned int i = 0; i != locations.size() - 1; ++i) {
			//mylog<< "location: "<<i<<"| ";
			for (unsigned int j = 0; j != transitions[i]; ++j) {
				//mylog<<mback(i,j)<<' ';
			}
			//mylog<<endl;
		}
		//mylog<<"forwardmargin:"<<endl;
		for (unsigned int i = 0; i != locations.size() - 1; ++i) {
			//mylog<< "location: "<<i<<"| ";
			for (unsigned int j = 0; j != transitions[i]; ++j) {
				//mylog<<mfore(i,j)<<' ';
			}
			//mylog<<endl;
		}
		//mylog<<"unary:"<<endl;
		for (unsigned int i = 0; i != locations.size(); ++i) {
			//mylog<< "location: "<<i<<"| ";
			for (unsigned int j = 0; j != (locations[i] + occ); ++j) { //for (unsigned int j = 0; j != locations[i]; ++j) { //CHANGE
				//mylog<<un(i,j)<<' ';
			}
			//mylog<<endl;
		}
	}

	void assign() {
		//mylog<<"clearmargin"<<endl;
		clearmargin(); //cout << "assign clear margin ok" << endl;
		//mylog<<"cleared margin"<<endl;
		for (unsigned int i = 0; i != locations.size(); ++i)
			update_unary_second_pre(i);

		for (int i = locations.size() - 2; i != -1; --i) {
			//mylog<<"backward point i"<<i<<endl;
			backward_point(i);
		}

		for (int i = 0; i != locations.size(); ++i) {
			//mylog<<"forward_set "<<i<<endl;
			forward_set(i);
		}
		//mylog<<"Labelling: ";
		for (unsigned int i = 0; i != locations.size(); ++i)
			//mylog<<labelling[i]<<' ';
			//mylog<<endl;

			for (int i = 0; i != locations.size(); ++i) {
				//mylog<<"update_unary second"<<endl;
				update_unary_second(i);
			}
	};

	void disp_label() {
		for (unsigned int i = 0; i != locations.size(); ++i) {
			//mylog<<labelling[i]<<' ';
		}
		//mylog<<endl;
	}
	~point() {
		for (unsigned int i = 0; i != locations.size() - 1; ++i) {
			//      //mylog<<"freeing marginbackward"<<i<<endl;
			free(marginbackward[i]);
			// //mylog<<"freeing marginforward"<<i<<endl;
			free(marginforward[i]);
		}

	}
};


class bundle {
public:

	vector<point> points;
	
	double** message;
	bundle(unsigned int tracks, const double** transitioncost, const int** transitionin, const  int** transitionout, vector<int>* notransitions, const double*** unary) { //, const double ** tertiary){
		points.resize(tracks);

		//Pairwise potentials
		for (unsigned int i = 0; i != tracks; ++i)
			for (unsigned int j = 0; j != locations.size() - 1; ++j) {
				points[i].transitions[j] = (*notransitions)[j]; //cerr << "transitions: "<< (*notransitions)[j] << endl;
				points[i].pairwise[j] = transitioncost[j]; //cerr << "pairwise: " << transitioncost[j] << endl;
				points[i].pairend[j] = transitionin[j]; //cerr << "pairend: " << transitionin[j] << endl;
				points[i].pairstart[j] = transitionout[j]; //cerr << "pairstart: " << transitionout[j] << endl;
														   //cerr << points[i].transitions[j] << " " << points[i].pairwise[j] << " " << points[i].pairend[j] << " " << endl;

			}
		for (unsigned int i = 0; i != tracks; ++i)
			for (unsigned int j = 0; j != locations.size() - 1; ++j) {
				points[i].marginbackward[j] = (double*)malloc(sizeof(double)*(*notransitions)[j]);
				points[i].marginforward[j] = (double*)malloc(sizeof(double)*(*notransitions)[j]);
			}
		//mylog<<"Assigned pairwise"<<endl;

		message = (double**)malloc(sizeof(double*)*locations.size());
		for (unsigned int j = 0; j != locations.size(); ++j) {
			message[j] = (double*)malloc(sizeof(double)*(locations[j] + occ)); //message[j] = (double*)malloc(sizeof(double)*locations[j]); //CHANGE
			for (unsigned int k = 0; k != (locations[j] + occ); ++k) {
				message[j][k] = 0;
			}
		}

		for (unsigned int i = 0; i != tracks; ++i)
			points[i].message = message;
		// Unary potentials
		for (unsigned int i = 0; i != tracks; ++i)
			points[i].unary = unary[i];
		//mylog<<"assigned unary"<<endl; 
		for (unsigned int i = 0; i != points.size(); ++i) {
			//mylog<<"NODE "<<i<<endl;
			points[i].inspect();
		}
	}
	void run() {
		//mylog<<"nop "<<points.size()<<endl;
		//mylog<<endl<<"initial margins"<<endl;
		//mylog<<"---------------"<<endl;
		//cout << "start run" << endl;
		for (unsigned int i = 0; i != points.size(); ++i) {
			//mylog<<"i: "<<i<<endl;
			points[i].margin(); //cout << "margin ok" << endl;
			points[i].disp_margin(); //cout << "disp margin ok" << endl;
		}
		//mylog<<endl<<"assigned margins"<<endl;
		//mylog<<"---------------"<<endl;
		for (int i = points.size() - 1; i != -1; --i) {
			//mylog<<"i: "<<i <<endl;
			points[i].assign(); //cout << "assign ok" << endl;
		}
		for (unsigned int i = 0; i != points.size(); ++i) {
			//mylog<<"i: "<<i <<endl;
			points[i].disp_margin(); //cout << "disp margin" << endl;
		}

		for (unsigned int i = 0; i != points.size(); ++i) {
			//mylog<<"Point "<<i<<"| ";
			points[i].disp_label(); //cout << "disp label" << endl;
		}

	}

	~bundle() {
		//mylog<<"Commencing bundle tidy up"<<endl;
		for (unsigned int i = 0; i != locations.size(); ++i) {
			//      //mylog<<"freeing message i: "<<i<<' '<<((unsigned long) message[i])<<endl;
			free(message[i]);
		}
		//    //mylog<<"freeing final message"<<endl;
		free(message);

	}

};

Mat match2nd(vector <MyMat> unary_costs, vector <MATSPARSE> pairwise_costs, int Nong, double occ_cost, double bam_tie, const unsigned int frames, const unsigned int points, const int *permutation);

double computeCostTrack(Mat& M_paw_bottom, vector <MyMat> unary_costs, vector <MATSPARSE> pairwise_costs, const int* permutation);

#endif