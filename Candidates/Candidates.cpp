#include "Candidates.hpp"

//--------- Candidate
Candidate::Candidate() {
	s = -1;
	p = Point_<int>(-1, -1);
}

Candidate::Candidate(int x, int y, double scr) {
	p = Point_<int>(x, y);
	s = scr;
}

Candidate::Candidate(Point_<int> P, double scr) {
	s = scr;
	p = P;
}

void Candidate::write(FileStorage& fs) const {
	fs << "{" << "Point_x" << p.x << "Point_y" << p.y << "Score" << s << "}";
}

void Candidate::read(const FileNode& node) {

	p.x = (int)node["Point_x"];
	p.y = (int)node["Point_y"];
	s = (double)node["Score"];

}


//Auxiliary functions for Candidate class:
bool compareCandidate(Candidate a, Candidate b) {
	//Using > returns candidates in descending order, as needed.
	return a.score() > b.score();
}


//---------- P22D
P22D::P22D() {
	CB = Candidate();

	yt.push_back(-1);
	st.push_back(-1);
}

P22D::P22D(int xc, int ybc, int ytc, double scr_b, double scr_t) {

	CB = Candidate(xc, ybc, scr_b);

	yt.push_back(ytc);
	st.push_back(scr_t);
}

P22D::P22D(Point_<int> Pb, Point_<int> Pt, double scr_b, double scr_t) {

	/*x = Pb.x;
	yb = Pb.y;
	sb = scr_b;
*/
	CB = Candidate(Pb, scr_b);

	yt.push_back(Pt.y);
	st.push_back(scr_t);
}

P22D::P22D(Candidate Cb, Candidate Ct) {
	//P22D(Cb.point(), Ct.point(), Cb.score(), Ct.score());

	//CB = Cb; //FIXME: Does this copy to a new place or just references?
	CB = Candidate(Cb.point(), Cb.score()); //If copy doesn't work, this one does.

	yt.push_back(Ct.point().y);
	st.push_back(Ct.score());
}

Point_<int> P22D::point_bottom() const{ 
	return CB.point(); 
}

double P22D::score_bottom() const{ 
	return CB.score(); 
}

int P22D::x_coord() const{ 
	return CB.point().x; 
}

int P22D::y_bottom_coord() const{ 
	return CB.point().y; 
}


void P22D::add_top_candidate(Candidate C) {
	add_top_candidate_safe(C.point().x, C.point().y, C.score());
};

void P22D::add_top_candidate(Point_<int> P, double s) {
	add_top_candidate_safe(P.x, P.y, s);
}

void P22D::add_top_candidate(int y, double s) {
	add_top_candidate_safe(CB.point().x, y, s);
}

void P22D::add_top_candidate_safe(int X, int Y, double S) {
	if (number_of_candidates() == 0) {
		yt[0] = Y;
		st[0] = S;
		return;
	}
	CV_Assert(S >= 0);
	yt.push_back(Y);
	st.push_back(S);
}


Point_<int> P22D::point_top(uint index) const{

	assert(index < yt.size());
	Point_<int> P(CB.point().x, yt[index]);
	return P;
}

int P22D::y_top_coord(uint index) const {

	assert(index < yt.size());
	return yt[index];
}

double P22D::score_top(uint index) const{

	assert(index < yt.size());
	return st[index];
}

Candidate P22D::get_candidate_top(uint index) const {

	assert(index < yt.size());
	return Candidate(CB.point().x, yt[index], st[index]);

}

Candidate P22D::get_candidate_bottom() const {
	return CB;
}

int P22D::number_of_candidates() const {

	//NOTE: Make sure there is always an element in st
	if (st[0] < 0)
		return 0;
	else
		return st.size();

}

void P22D::write(FileStorage& fs) const {
	fs << "{" << "Candidate_bottom" << CB;
	fs << "n_candidates_top" << (int) yt.size(); //This should be number_of_candidates() but that has some problem with const...
	fs << "Candidates_top" << "[:";
	for (int i = 0; i < yt.size(); ++i) {
		fs << yt[i];
	}
	fs << "]";
	
	fs << "Scores_top" << "[:";
	for (int i = 0; i < st.size(); ++i) {
		fs << st[i];
	}
	fs << "]";
	fs << "}";

}

void P22D::read(const FileNode& node) {
	node["Candidate_bottom"] >> CB;
	
	FileNodeIterator it = node["Candidates_top"].begin(), it_end = node["Candidates_top"].end();
	vector<int> YT;
	for (; it != it_end; ++it) {
		int y = (int)*it;
		assert(y >= 0);
		YT.push_back(y);
	}

	it = node["Scores_top"].begin(), it_end = node["Scores_top"].end();
	vector<double> ST;
	for (; it != it_end; ++it) {
		
		double s = (double)*it;
		ST.push_back(s);
	}
	assert(ST.size() == YT.size());

	for (int i = 0; i < YT.size(); ++i) {
		add_top_candidate(YT[i], ST[i]);
	}

}