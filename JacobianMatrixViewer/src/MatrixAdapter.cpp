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
