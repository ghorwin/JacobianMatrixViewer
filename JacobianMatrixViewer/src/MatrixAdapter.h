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

		/*! Indicates that is cell is used and has a value different from zero. */
		CS_Used,
		/*! Indicates that is cell is not part of the matrix pattern. */
		CS_Unused,
		/*! Indicates that is cell is used and has a value of zero. */
		CS_Zero,

		/*! One of the matrix has Unused value, the other one has a value different from zero (this definitely present).
			This indicates a pattern error in the matrix with smaller pattern.
		*/
		CS_DifferentByUsage,

		/*! One of the matrixes has Unused value, the other one has a zero value. This may indicate a pattern error
			but is likely the cause when the more filled pattern just computed a zero where there is no entry. */
		CS_MayBeDifferentByUsage,

		/*! Both matrixes have a used cell, but different values. */
		CS_DifferentByValue,
		/*! Both matrixes have a used cell, but slightly different values (1e-9) */
		CS_SlightlyDifferentByValue
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

	/*! Evaluates difference between both values by used a relative and absolute threshold. */
	bool nearlyEqual(double val1, double val2) const;

	static double				m_relTol;
	static double				m_absTol;
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


class ComparisonMatrixAdapter : public AbstractMatrixAdapter {
public:

	// AbstractMatrixAdapter interface
	unsigned int dimension() const override;
	CellState state(unsigned int i, unsigned int j) const override;
	double value(unsigned int i, unsigned int j) const override;

	SingleMatrixAdapter	*m_first = nullptr;
	SingleMatrixAdapter	*m_second = nullptr;
};

#endif // MATRIXADAPATER_H
