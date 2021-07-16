#ifndef MATRIXADAPATER_H
#define MATRIXADAPATER_H

namespace IBKMK {
	class SparseMatrixCSR;
//	class SparseMatrixEID;
	class DenseMatrix;
//	class BandMatrix;
}

/*! Holds data of a matrix and provides it conveniently to viewer classes. */
class MatrixAdapter {
public:
	MatrixAdapter();

	/*! Releases memory. */
	~MatrixAdapter();

	void clear();

	IBKMK::SparseMatrixCSR		*m_sparseMatrixCSR = nullptr;
//	IBKMK::SparseMatrixEID		*m_sparseMatrixEID = nullptr;
	IBKMK::DenseMatrix			*m_denseMatrix = nullptr;

};


#endif // MATRIXADAPATER_H
