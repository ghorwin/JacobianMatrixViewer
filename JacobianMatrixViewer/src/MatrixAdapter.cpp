#include "MatrixAdapter.h"

#include <IBKMK_SparseMatrixCSR.h>
#include <IBKMK_DenseMatrix.h>

const double LIMIT = 1e-12;

SingleMatrixAdapter::SingleMatrixAdapter() {

}


SingleMatrixAdapter::~SingleMatrixAdapter() {
	clear();
}


void SingleMatrixAdapter::clear() {
	delete m_sparseMatrixCSR;
	m_sparseMatrixCSR = nullptr;
	delete m_denseMatrix;
	m_denseMatrix = nullptr;
}


unsigned int SingleMatrixAdapter::dimension() const {
	if (m_sparseMatrixCSR != nullptr)
		return m_sparseMatrixCSR->n();
	else if (m_denseMatrix != nullptr)
		return m_denseMatrix->n();
	else
		return 0;
}


SingleMatrixAdapter::CellState SingleMatrixAdapter::state(unsigned int i, unsigned int j) const {
	if (m_sparseMatrixCSR != nullptr) {
		unsigned int colIndx = m_sparseMatrixCSR->storageIndex(i,j);
		if (colIndx == m_sparseMatrixCSR->nnz())
			return CS_Unused;
		else
			return CS_Used;
	}
	else if (m_denseMatrix != nullptr) {
		if (std::fabs(value(i,j)) < LIMIT )
			return CS_Zero;
		else
			return CS_Used;
	}
	else
		return CS_Unused;
}


double SingleMatrixAdapter::value(unsigned int i, unsigned int j) const {
	if (m_sparseMatrixCSR != nullptr) {
		return m_sparseMatrixCSR->value(i,j);
	}
	else if (m_denseMatrix != nullptr) {
		return m_denseMatrix->value(i,j);
	}
	else
		return 0;
}


unsigned int ComparisonMatrixAdapter::dimension() const {
	if (m_first == nullptr || m_second == nullptr)
		return 0;
	else if (m_first->dimension() != m_second->dimension())
		return 0;
	else
		return m_first->dimension();
}


ComparisonMatrixAdapter::CellState ComparisonMatrixAdapter::state(unsigned int i, unsigned int j) const {
	if (m_first == nullptr || m_second == nullptr)
		return CS_Unused;

	// state of first
	CellState f1 = m_first->state(i,j);
	CellState f2 = m_second->state(i,j);

	if (f1 == f2) {
		if (f1 == CS_Used || f2 == CS_Used) {
			double diff = std::fabs(m_first->value(i,j) - m_second->value(i,j));
			if (diff < 1e-5)
				return CS_SlightlyDifferentByValue;
			else
				return CS_DifferentByValue;
		}
		return f1;
	}

//	/*! One of the matrix has Unused value, the other one has a value different from zero (this definitely present).
//		This indicates a pattern error in the matrix with smaller pattern.
//	*/
//	CS_DifferentByUsage,

//	/*! One of the matrixes has Unused value, the other one has a zero value. This may indicate a pattern error
//		but is likely the cause when the more filled pattern just computed a zero where there is no entry. */
//	CS_MayBeDifferentByUsage,

//	/*! Both matrixes have a used cell, but different values. */
//	CS_DifferentByValue

	if (f1 == CS_Unused) {
		if (f2 == CS_Used)
			return CS_DifferentByUsage;
		else /* if (f2 == CS_Zero) */
			return CS_MayBeDifferentByUsage;
	}
	if (f2 == CS_Unused) {
		if (f1 == CS_Used)
			return CS_DifferentByUsage;
		else /* if (f1 == CS_Zero) */
			return CS_MayBeDifferentByUsage;
	}

	IBK_ASSERT(false);
//	if (m_first->value(i,j) != m_second->value(i,j))
	return CS_DifferentByValue;
}


double ComparisonMatrixAdapter::value(unsigned int i, unsigned int j) const {
	CellState s = state(i,j);
	switch (s) {
		case AbstractMatrixAdapter::CS_Used:
		case AbstractMatrixAdapter::CS_DifferentByValue:
		case AbstractMatrixAdapter::CS_SlightlyDifferentByValue:
			return m_first->value(i,j) - m_second->value(i,j);
		default: ;
	}
	return 0;
}
