#ifndef MATRIXADAPATER_H
#define MATRIXADAPATER_H

namespace IBKMK {
	class SparseMatrixCSR;
//	class SparseMatrixEID;
	class DenseMatrix;
//	class BandMatrix;
}

/*! Defines the interface to access matrix data. */
class AbstractMatrixAdapter {
public:

	/*! The cell state returned for each cell. */
	enum CellState {
		CS_Used,
		CS_Unused,
		CS_Zero,
		CS_DifferentByUsage,
		CS_DifferentByValue
	};

	/*! Releases memory. */
	virtual ~AbstractMatrixAdapter() {}

	/*! Returns dimension of matrix. */
	virtual unsigned int dimension() const = 0;

	/*! Returns cell state, for row i and column j. */
	virtual CellState state(unsigned int i, unsigned int j) const = 0;

	/*! Returns the value of the cell. For all cells that are unused,
		0 is returned. For different cell values, the differences is returned.
	*/
	virtual double value(unsigned int i, unsigned int j) const = 0;
};


/*! Holds data of a matrix and provides it conveniently to viewer classes.
	Owns the matrix.
*/
class SingleMatrixAdapter : public AbstractMatrixAdapter {
public:
	SingleMatrixAdapter();

	/*! Releases memory. */
	~SingleMatrixAdapter();

	void clear();

	// AbstractMatrixAdapter interface
	unsigned int dimension() const override;
	CellState state(unsigned int i, unsigned int j) const override;
	double value(unsigned int i, unsigned int j) const override;

	IBKMK::SparseMatrixCSR		*m_sparseMatrixCSR = nullptr;
//	IBKMK::SparseMatrixEID		*m_sparseMatrixEID = nullptr;
	IBKMK::DenseMatrix			*m_denseMatrix = nullptr;

};



#endif // MATRIXADAPATER_H
