//MyMat.h
#ifndef __MyMat_H_INCLUDED__
#define __MyMat_H_INCLUDED__


//==== Forward declarations:

//==== Includes:
#include <opencv2/core.hpp>
#include <iostream>
#include <assert.h>
#include <vector>
#include <string>
#include <fstream>

//==== Definitions:
using namespace std;
using namespace cv;

class MyMat {
//Simple implementation of a matrix:
	double* values;
	int nrows;
	int ncols;
	int numel;
public:
	

	MyMat();
	MyMat& operator=(const MyMat &M); //Assignment constructor;
	MyMat(const MyMat &M); //Copy constructor;
	~MyMat();
	MyMat(unsigned int, unsigned int);
	
	void put(unsigned int, unsigned int, double);
	double get(unsigned int, unsigned int) const;
	inline int Nrows() const { return nrows; };
	inline int Ncols() const { return ncols; };
	inline int Numel() const { return numel; };
	inline double* getValues() const { return values; };

	//Read/Write using yml opencv-like:
	void write(FileStorage& fs) const;
	void read(const FileNode& node);

};

ostream& operator<< (ostream &out, MyMat const &M);

static void write(FileStorage& fs, const string&, const MyMat& c) {
	c.write(fs);
}

static void read(const FileNode& node, MyMat& c, const MyMat& default_value = MyMat()) {

	if (node.empty()) {
		c = default_value;
	}
	else {
		c.read(node);
	}
}

class MATSPARSE {
	//Emulating MATLAB sparse matrix:
	int* Ir = 0;
	int* Jc = 0;
	double* Pr = 0;
	int nzel = 0;
	int n_rows = 0;
	int n_cols = 0;

public:
	MATSPARSE();
	~MATSPARSE();
	MATSPARSE(const MyMat* M);
	MATSPARSE(const MATSPARSE &M); //copy constructor
	MATSPARSE(MATSPARSE &M); //move constructor
	MATSPARSE& operator=(const MATSPARSE &M); //copy assignment
	MATSPARSE& operator=(MATSPARSE &&M); //move assignment

	double get(int, int);
	inline int* getIr() const { return Ir; };
	inline int* getJc() const { return Jc; };
	inline double* getPr() const { return Pr; };
	inline int nz() const { return nzel; };
	inline int Nrows() const { return n_rows; };
	inline int Ncols() const { return n_cols; }
	inline void print_contents(string filename);

	//Read/Write using yml opencv-like:
	void write(FileStorage& fs) const;
	//void read(const FileNode& node);

};

ostream& operator<< (ostream &out, MATSPARSE const &M);


//class MATSPARSE {
////Emulating MATLAB sparse matrix:
//	int* Ir = 0;
//	int* Jc = 0;
//	double* Pr = 0;
//	int nzel = 0;
//	int n_rows = 0;
//	int n_cols = 0;
//
//public:
//	MATSPARSE();
//	~MATSPARSE();
//	MATSPARSE(MyMat*);
//	//MATSPARSE(const Mat &M);
//	MATSPARSE(const MATSPARSE &M); //copy constructor
//	MATSPARSE(MATSPARSE &M); //move constructor
//	MATSPARSE& MATSPARSE::operator=(const MATSPARSE &M); //assignment operator
//	MATSPARSE& MATSPARSE::operator=(MATSPARSE &&M); //move asignment
//
//	double get(int, int);
//	inline int* getIr() const { return Ir; };
//	inline int* getJc() const { return Jc; };
//	inline double* getPr() const { return Pr; };
//	inline int nz() const { return nzel; };
//	inline int Nrows() const { return n_rows; };
//	inline int Ncols() const { return n_cols; }
//	inline void print_contents(string filename);
//
//	//Read/Write using yml opencv-like:
//	void write(FileStorage& fs) const;
//	//void read(const FileNode& node);
//
//};
//
//ostream& operator<< (ostream &out, MATSPARSE const &M);

static void write(FileStorage& fs, const string&, const MATSPARSE& c) {
	c.write(fs);
}

//static void read(const FileNode& node, MATSPARSE& c, const MATSPARSE& default_value = MATSPARSE()) {
//
//	if (node.empty()) {
//		c = default_value;
//	}
//	else {
//		c.read(node);
//	}
//}

#endif

