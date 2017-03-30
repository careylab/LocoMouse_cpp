#include "MyMat.hpp"

//class MyMat:
MyMat::MyMat() {
	nrows = 0;
	ncols = 0;
	numel = 0;
	values = NULL;
}

MyMat& MyMat::operator=(const MyMat &M) {

	nrows = M.Nrows();
	ncols = M.Ncols();
	numel = M.Numel();

	values = new(nothrow) double[numel];

	const double* Val = M.getValues();

	for (unsigned int i = 0; i < numel; ++i) {
		values[i] = Val[i];
	}

	return *this;
}

//MyMat copy constructor:
MyMat::MyMat(const MyMat &M) {

	nrows = M.Nrows();
	ncols = M.Ncols();
	numel = M.Numel();

	values = new(nothrow) double[numel];

	const double* Val = M.getValues();

	for (unsigned int i = 0; i < numel; ++i) {
		values[i] = Val[i];
	}
}

MyMat::MyMat(unsigned int n, unsigned int m) {
	nrows = n;
	ncols = m;
	numel = n*m;
	double* v = new(nothrow) double[numel];
	values = v;
	for (unsigned int i = 0; i < numel; ++i) {
		values[i] = 0;
	}
}

void MyMat::put(unsigned int i, unsigned int j, double val) {
	assert(i < nrows);
	assert(j < ncols);
	values[j*nrows + i] = val;
}

MyMat::~MyMat() {
	if (values) {
		delete[] values;
	}
}

double MyMat::get(unsigned int i, unsigned int j) const {
	assert(i < nrows);
	assert(j < ncols);
	return values[j*nrows + i];
}

void MyMat::write(FileStorage& fs) const {
	
	fs << "{";
	fs << "n_rows" << nrows;
	fs << "n_cols" << ncols;
	fs << "data" << "[:";
	for (int iel = 0; iel < numel; iel++) {
		fs << values[iel];
	}
	fs << "]";
	fs << "}";
	return;
}

void MyMat::read(const FileNode& node) {
	return;
}

ostream& operator<< (ostream &out, MyMat const &M) {

	if (M.Nrows() == 0 | M.Ncols() == 0) {
		out << "Matrix is empty!" << endl;
		return out;
	}

	out << "[";
	for (unsigned int i_row = 0; i_row < M.Nrows(); i_row++) {

		for (unsigned int i_col = 0; i_col < (M.Ncols() - 1); i_col++) {
			out << M.get(i_row, i_col) << ", ";
		}

		out << M.get(i_row, M.Ncols() - 1);

		if (i_row != M.Nrows() - 1) {
			out << ";" << endl;
		}
		else {
			out << "]" << endl;
		}
	}
	return out;
}


//class MATSPARSE
//class MATSPARSE
MATSPARSE::MATSPARSE() {
	int* Ir = 0;
	int* Jc = 0;
	double* Pr = 0;
	int nzel = 0;
	int n_rows = 0;
	int n_cols = 0;
}


MATSPARSE::MATSPARSE(const MyMat* M) {
//	cerr << "MyMat constructor" << endl;
	// Saves a matrix in sparse storage following MATLAB's sparse matrix conventions
	vector<int> rows, jc_vec;
	jc_vec.reserve(M->Ncols() + 1);
	vector<double> values;
	double get_val;
	nzel = 0;
	for (unsigned int j = 0; j < M->Ncols(); ++j) {
		for (unsigned int i = 0; i < M->Nrows(); ++i) {
			get_val = M->get(i, j);
			if (get_val != 0) {
				rows.push_back((int)i);
				values.push_back(get_val);
				nzel += 1;
			}
		}
		jc_vec.push_back(nzel);
	}

	//NOTE: ir, jc, and val are implemented this way so they will interface properly with the match2nd code, which was written in mex. This can create memory leaks if the vectors are not deleted properly! 
	int* ir = new (nothrow) int[rows.size()];
	int* jc = new (nothrow) int[jc_vec.size() + 1];
	double* val = new(nothrow) double[rows.size()];

	copy(rows.begin(), rows.end(), ir);
	copy(values.begin(), values.end(), val);

	jc[0] = 0;
	copy(jc_vec.begin(), jc_vec.end(), &jc[1]);

	Ir = ir;
	Pr = val;
	Jc = jc;
	n_rows = M->Nrows();
	n_cols = M->Ncols();

}

MATSPARSE::MATSPARSE(const MATSPARSE &M) {
	//cerr << "Copy constructor" << endl;
	nzel = M.nzel;
	n_rows = M.n_rows;
	n_cols = M.n_cols;

//	cerr << "nzel n_rows n_cols: " << nzel << " " << n_rows << " " << n_cols << endl;

	int* ir = new (nothrow) int[nzel];
	int* jc = new (nothrow) int[n_cols + 1];
	double* val = new(nothrow) double[nzel];

	for (unsigned int i = 0; i < nzel; ++i) {
		ir[i] = M.Ir[i];
		val[i] = M.Pr[i];
	}

	for (unsigned int i = 0; i < n_cols + 1; ++i) {
		jc[i] = M.Jc[i];
	}

	Ir = ir;
	Jc = jc;
	Pr = val;

	return;
}

MATSPARSE::MATSPARSE(MATSPARSE &M) {
//	cerr << "Move constructor" << endl;
	nzel = M.nzel;
	n_rows = M.n_rows;
	n_cols = M.n_cols;

	Ir = M.Ir;
	Jc = M.Jc;
	Pr = M.Pr;

	//reset incoming class
	M.nzel = 0;
	M.n_rows = 0;
	M.n_cols = 0;

	M.Ir = nullptr;
	M.Pr = nullptr;
	M.Jc = nullptr;

	return;
}

MATSPARSE& MATSPARSE::operator=(const MATSPARSE &M) {
//	cerr << "Copy assign operator:" << endl;
	nzel = M.nzel;
	n_rows = M.n_rows;
	n_cols = M.n_cols;

	int* ir = new (nothrow) int[nzel];
	int* jc = new (nothrow) int[n_cols + 1];
	double* val = new(nothrow) double[nzel];

	for (unsigned int i = 0; i < nzel; ++i) {
		ir[i] = M.Ir[i];
		val[i] = M.Pr[i];
	}

	for (unsigned int i = 0; i < n_cols + 1; ++i) {
		jc[i] = M.Jc[i];
	}

	Ir = ir;
	Jc = jc;
	Pr = val;

	return *this;

}

MATSPARSE& MATSPARSE::operator=(MATSPARSE &&M) {
//	cerr << "Move assign" << endl;
	if (this != &M) {

		//Freeing current resources:
		delete[] Ir;
		delete[] Jc;
		delete[] Pr;
		//nzel = 0;
		//n_rows = 0;
		//n_cols = 0;

		//Copying resources:
		nzel = M.nzel;
		n_rows = M.n_rows;
		n_cols = M.n_cols;

		Ir = M.Ir;
		Jc = M.Jc;
		Pr = M.Pr;

		//Resetting M:
		M.Ir = nullptr;
		M.Jc = nullptr;
		M.Pr = nullptr;
	}
	return *this;

};


MATSPARSE::~MATSPARSE() {
	//cerr << "MATSPARSE Destructor" << endl;

	if (Ir != nullptr) {
		//cerr << "Deleting Ir" << endl;
		//cerr << Ir << endl;
		delete[] Ir;
	}

	if (Jc != nullptr) {
		//cerr << "Deleting Jc" << endl;
		//cerr << Jc << endl;
		delete[] Jc;
	}
	if (Pr != nullptr) {
		//cerr << "Deleting Pr" << endl;
		//cerr << Pr << endl;
		delete[] Pr;
	}

}

void MATSPARSE::write(FileStorage& fs) const {
	
	vector <int> i_cols(nzel), i_rows(nzel);

	fs << "{";
	//Data:
	fs << "data" << "[:";
	int curr_index, n_rows_per_col;
	for (int icol = 0; icol < n_cols; icol++) {
		
		n_rows_per_col = Jc[icol + 1] - Jc[icol];
		
		for (int irow = 0; irow < n_rows_per_col; irow++) {

			curr_index = Jc[icol] + irow;
		
			i_cols[curr_index] = icol;
			i_rows[curr_index] = Ir[curr_index];
			fs << Pr[Jc[icol] + irow];
		}
	}
	fs << "]";
		
	//Row indices:
	fs << "row_index" << "[:";
	for (int i_row = 0; i_row < nzel; i_row++) {
		fs << i_rows[i_row];
	}
	fs << "]";

	//Col indices:
	fs << "col_index" << "[:";
	for (int i_col = 0; i_col < nzel; i_col++) {
		fs << i_cols[i_col];
	}
	fs << "]";
	fs << "}";

	return;
}

//void MATSPARSE::read(const FileNode& node) {
//	return;
//}


void MATSPARSE::print_contents(string file_name) {

	ofstream s;
	s.open(file_name.c_str());
	s << "Matrix size: " << n_rows << ", " << n_cols << endl;
	//DEBUG: This debug does not work as Jc is not read as programmed. See setJc in matlab mex documentation!
	for (int icol = 0; icol < n_cols; icol++) {
		int n_rows_per_col = Jc[icol + 1] - Jc[icol];
		for (int irow = 0; irow < n_rows_per_col; irow++) {
			s << "(" << Ir[Jc[icol] + irow] << "," << icol << ") = " << Pr[Jc[icol] + irow] << endl;
		}
	}
	s.close();
}

double MATSPARSE::get(int irow, int icol) {
	//Returns element at position (irow,icol):
	double val = 0;
	return val;
	assert(icol < n_cols);
	assert(irow < n_rows);

	int n_rows_per_col = Jc[icol + 1] - Jc[icol];

	if (n_rows_per_col > 0) {

		for (int i = 0; i < n_rows_per_col; ++i) {
			if (Ir[Jc[icol] + i] == irow) {
				val = Pr[Jc[icol] + irow];
			}
		}

	}
	return val;
}

ostream& operator<< (ostream &out, MATSPARSE const &M) {

	if (M.nz() == 0) {
		out << "Matrix is empty!" << endl;
		return out;
	}

	const int* Ir = M.getIr();
	const int* Jc = M.getJc();
	const double* Pr = M.getPr();

	for (int icol = 0; icol < M.Ncols(); ++icol) {

		int n_rows_per_col = Jc[icol + 1] - Jc[icol];

		for (int irow = 0; irow < n_rows_per_col; ++irow) {
			out << "(" << Ir[Jc[icol] + irow] << "," << icol << ") = " << Pr[Jc[icol] + irow] << endl;
		}
	}
}