#ifndef MatrixInterfaceBandH
#define MatrixInterfaceBandH

#include <IBK_BandMatrix.h>
#include <limits>

/*! Implementation of an interface to the IBK::BandMatrix.
	This class can be used as policy-class for the MatrixVisualizer.
*/
class MatrixInterfaceBand {
public:

	/*! Sets a new matrix content to the matrix interface. */
	void setMatrix(const IBK::BandMatrix & matrix) {
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
		if (col+m_matrix.ml()<row || col>row+m_matrix.mu())
			return std::numeric_limits<double>::infinity();
		else
			return m_matrix(row, col);
	}

	/*! Storage member, holds matrix data to be used for drawing matrix. */
	IBK::BandMatrix	m_matrix;
};

#endif // MatrixInterfaceBandH
