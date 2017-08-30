//Candidates.hpp
#ifndef __Candidates_H_INCLUDED__
#define __Candidates_H_INCLUDED__

//Forward declarations:

//Includes:
#include <opencv2/core.hpp>
#include <vector>
#include <iostream>
#include <assert.h>
using namespace std;
using namespace cv;

//Candidate class:
class Candidate {
public:
	Point_<int> p;
	double s;
public:
	Candidate();
	Candidate(int, int, double);
	Candidate(Point_<int>, double);

	//FIXME: Remove these methods and just access the stuff directly:
	inline Point_<int> point() const { return p; };
	inline double score() const { return s; };
	inline void set_score(double new_s) { s = new_s; }; 

	//Read/Write using yml opencv-like:
	void write(FileStorage& fs) const;
	void read(const FileNode& node);

};

//Auxiliary functions
bool compareCandidate(Candidate a, Candidate b);

//FIXME: I don't understand why these need to be here as static but otherwise it doesn't compile. Probably conflict with the generic << operator and such.
static void write(FileStorage& fs, const string&, const Candidate& c) {
	c.write(fs);
}

static void read(const FileNode& node, Candidate& c, const Candidate& default_value = Candidate()) {
	
	if (node.empty()) {
		c = default_value;
	}
	else {
		c.read(node);
	}
} 

static ostream& operator << (ostream& out, Candidate c) {
	out << "[(" << c.point().x << ", ";
	out << c.point().y << ") with ";
	out << "score = " << c.s << "]";
	return out;
};

//2D candidate with multiple Z candidates:
//Create the 2x2D point structure:
class P22D {
private:
	//int x; //Horizontal position;
	//int yb; //Vertical position on bottom image;
	//double sb; //Score on bottom image.

	Candidate CB;
	
	vector<int> yt; //Vertical positon(s) on top image;
	vector<double> st; //Score(s) on top image.

public:
	//Constructor for when the data is empty.
	P22D();
	//~P22D();
	P22D(int, int, int, double, double);
	P22D(Point_<int>, Point_<int>, double, double);
	P22D(Candidate, Candidate);

public:
	Point_<int> point_bottom() const;
	Point_<int> point_side(uint) const;
	double score_bottom() const;
	double score_side(uint) const;
	int x_coord() const;
	int y_bottom_coord() const;
	int y_side_coord(uint) const;

	void add_side_candidate(Candidate);
	void add_side_candidate(Point_<int>, double);
	void add_side_candidate(int y, double s);
	
	int number_of_candidates() const;
	Candidate get_candidate_side(uint) const;
	Candidate get_candidate_bottom() const;

	//Read/Write using yml opencv-like:
	void write(FileStorage& fs) const;
	void read(const FileNode& node);

private:
	void add_side_candidate_safe(int x, int y, double s);
};

//Auxiliary functions
//FIXME: I don't understand why these need to be here as static but otherwise it doesn't compile. Probably conflict with the generic << operator and such.
static void write(FileStorage& fs, const string&, const P22D& c) {
	c.write(fs);
}

static void read(const FileNode& node, P22D& c, const P22D& default_value = P22D()) {

	if (node.empty()) {
		c = default_value;
	}
	else {
		c.read(node);
	}
}

static ostream& operator << (ostream& out, P22D c) {
	out << "Bottom candidate: " << c.get_candidate_bottom() << endl;
	out << c.number_of_candidates() << " top candidate(s):" << endl;
	for (uint i = 0; i < c.number_of_candidates(); ++i) {
		out << "[" << c.y_side_coord(i) << " with score = " << c.score_side(i) << "]" << endl;
	}
	return out;
};

#endif