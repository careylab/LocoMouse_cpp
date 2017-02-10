#include "match2nd.h"

//Auxiliary code for match2nd:
vector<int> locations;
int occ;
double occ_score;
double BAM;
ofstream mylog;


Mat match2nd(vector <MyMat> unary_costs, vector <MATSPARSE> pairwise_costs, int Nong, double occlusion_point_cost, double bam_tie, const unsigned int frames, const unsigned int points, const int *permutation) {

	occ = Nong;
	BAM = bam_tie;
	occ_score = occlusion_point_cost;
	Mat T = Mat::zeros(points, frames, CV_32SC1);

	//mylog.open("./log.txt");
	//mylog << "starting log" << endl;
	//mylog << "number of frames: " << frames << endl;
	//mylog << "number of points: " << points << endl;
 
	if ((frames < 2) || (points < 1)) {
		cout << "There must be at least one point and 2 frames." << endl;
		return T;
	}

	const double*** unary = (const double***)malloc(sizeof(const double**)*(points));
	for (unsigned int i = 0; i != points; ++i)
		unary[i] = (const double**)malloc(sizeof(const double*)*frames);

	//mylog<<"completed unary malloc"<<endl;

	locations.resize(frames);

	//mylog<<"Loading unary potentials"<<endl;

	//FIXME: Minimizing work at interface. If this project goes ahead, make sure the data that is computed at every frame is directly usable by the tracker!
	
	for (unsigned int j = 0; j != frames; ++j)
	{
		//mylog << "frame " << j << endl;

		//un is an elements * locations matrix
		unsigned int elements = unary_costs[j].Ncols();
		//mylog << "Checking number of columns: " << elements << endl;

		if (elements != points) {
			cout << "Wrong form of unary potentials." << elements << "!=" << points << endl;
			return T;
		}

		locations[j] = unary_costs[j].Nrows();
		//mylog << "Checking number of rows: " << locations[j] << endl;

		//bind data
		const double* matrix = unary_costs[j].getValues();
		//mylog << "Pointing matrix to the right location." << endl;

		for (unsigned int i = 0; i != points; ++i) {
			unary[i][j] = (matrix + permutation[i] * locations[j]); //Shifting by the number of rows.
		}
		//mylog << "Copied values to internal unary array." << endl;
	}
	//mylog << "memory copied" << endl;
	//DEBUG: Spying the contents of unary:
	/*for (int i = 0; i < points; ++i) {
		for (int j = 0; j < frames; ++j) {
			for (int k = 0; k < 1; ++k) {
				//mylog << "unary for point " << i << " and frame " << j << " is: " << unary[i][j][k] << " " << "permutation was: " << permutation[i] << endl;
			}
		}
	}*/

	vector <int> notrans(frames - 1); //Vector just to keep what was before (JF).
									 //Edited to conform with the LocoMouse code:
	//mylog << "notrans done" << endl;
	const  int** transin = new (nothrow) const int*[frames - 1]; //mylog << "trasin done" << endl;
	const  int** transout = new (nothrow) const int*[frames - 1]; //mylog << "trasin done" << endl;
	const  double** cost = new (nothrow) const double*[frames - 1]; //mylog << "trasin done" << endl;

	//mylog << "Made it outside of the loop!" << endl;

	for (unsigned int i = 0; i != frames - 1; ++i) {
		//Checking sizes: FIXME: Implement the N_rows and N_cols for sparse emulated matrices.

		if (pairwise_costs[i].Nrows() != locations[i + 1]+occ) {
			//mylog << "Comparison frame " << i <<": " << (pairwise_costs[i].Nrows()) << " "  << (locations[i + 1] + occ) << endl;
			//mylog << "pairwise dimensions do not match those of unary potentials" << endl;
			//mylog << "Expected " << locations[1 + i] << " candidates and " << occ << " occlusion points totalling "  << locations[i + 1] + occ << ", and found " << pairwise_costs[i].Nrows() << " rows." << endl;
			return T;
		}

		if (pairwise_costs[i].Ncols() != (locations[i] + occ)) {
			//mylog << "Comparison frame " << i << ":  " << pairwise_costs[i].Ncols() << " " << (locations[i] + occ) << endl;
			//mylog << "pairwise dimensions do not match those of unary potentials" << endl;
			//mylog << "Expected " << locations[i] << " candidates and " << occ << " occlusion points totalling " << locations[i] + occ << ", and found " << pairwise_costs[i].Ncols() << " columns." << endl;
			return T;
		}


		if (i == 0) {
			//mylog << "Values for pairwise matrix at frame " << i << endl;
			int nz = pairwise_costs[i].nz();
			int *row = pairwise_costs[i].getIr();
			double *val = pairwise_costs[i].getPr();

			//mylog << "There are " << nz << " pairwise values:" << endl;
			for (int inz = 0; inz < nz; inz++) {
				//mylog << "row, value: " << row[inz] << " " << val[inz] << endl;
			}

		}


		transin[i] = pairwise_costs[i].getIr(); //Row index of non-zero elements.
		transout[i] = pairwise_costs[i].getJc(); //Jc is a 1x(N+1) vector with the number of elements per row (Jc[0] is always 0);
		cost[i] = pairwise_costs[i].getPr(); //Values
		notrans[i] = pairwise_costs[i].nz();//Number of non-zero elements in the matrix.

		//mylog << "Pointers assigned." << endl;


		//DEBUG:
		if (i == 0) {
			//mylog << "Checking values:" << endl;
			//mylog << transin[i][0] << endl;
			//mylog << transout[i][0] << endl;
			//mylog << cost[i][0] << endl;
			//mylog << notrans[i] << endl;
		}

	}

	//mylog << "Running algorithm..." << endl;
	bundle a(points, cost, transin, transout, &notrans, unary); // , (tert));
	a.run();
	//mylog << "run ok" << endl;
	
	for (unsigned int i = 0; i != points; ++i) {
		//mylog<<"freeing unary "<<i<<" "<< (unsigned long)unary[i]<<endl;
		free(unary[i]);
	}
	
	//mylog << "Copying the output" << std::endl;
	for (unsigned int i = 0; i < T.rows; ++i) {
		int* p = T.ptr<int>(i);
		for (unsigned int j = 0; j < T.cols; ++j)
			p[j] = a.points[i].labelling[j];
	}
	//mylog << "copy ok" << endl;
	free(unary);

	delete transin;
	delete transout;
	delete cost;

	return T;
	//mylog.close();
}

double computeCostTrack(Mat& M_paw_bottom, vector <MyMat> unary_potential_paw_bottom, vector <MATSPARSE> pairwise_potential_paw_bottom, const int* permutation) {

	double c = 0;
	
	int N_frames = M_paw_bottom.cols;
	int N_tracks = M_paw_bottom.rows;
	
	double un, pairwise;

	for (int i_tracks = 0; i_tracks < 4; ++i_tracks) {
		int *pM = M_paw_bottom.ptr<int>(i_tracks);
	
		for (int i_frames = 0; i_frames < N_frames; ++i_frames, pM += 1) {
			//Suming the unary term:
			if (pM[0] < unary_potential_paw_bottom[i_frames].Nrows()) {
				un = unary_potential_paw_bottom[i_frames].get(pM[0], permutation[i_tracks]);
			}
			else {
				un = 0; //occ_cost! FIXME;
			}
			c += un;
			//Suming the pairwise term:
			if (i_frames < (N_frames - 1)) {
				c += pairwise_potential_paw_bottom[i_frames].get(pM[1], pM[0]);
			}
		}

	}
	return c;
}