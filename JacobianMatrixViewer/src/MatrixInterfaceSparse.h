#ifndef MatrixInterfaceSparseH
#define MatrixInterfaceSparseH

#include <IBKMK_SparseMatrixCSR.h>
#include <limits>

/*! Implementation of an interface to the IBK::SparseMatrix.
	This class can be used as policy-class for the MatrixVisualizer.
*/
class MatrixInterfaceSparse {
public:

	/*! Sets a new matrix content to the matrix interface. */
	void setMatrix(const IBKMK::SparseMatrixCSR & matrix) {
		m_matrix = matrix;
	}

	/*! Returns size/dimension of matrix. */
	unsigned int size() const {
		return m_matrix.n();
	}

	/*! Returns cell value or std::numeric_limits<double>::infinity() if cell is not part of
		the matrix structure.
	*/
	double value(unsigned int row, unsigned int col) const {
		// try to find index of column in row
		unsigned int index = m_matrix.storageIndex(row, col);
		if (index == m_matrix.n()*m_matrix.elementsPerRow())
			return std::numeric_limits<double>::infinity();
		else
			return m_matrix.data()[index];
	}

	/*! Storage member, holds matrix data to be used for drawing matrix. */
	IBKMK::SparseMatrixCSR	m_matrix;
};

#endif // MatrixInterfaceSparseH
