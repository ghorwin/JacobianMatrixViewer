#include "MatrixAdapter.h"

#include <IBKMK_SparseMatrixCSR.h>
#include <IBKMK_DenseMatrix.h>

MatrixAdapter::MatrixAdapter() {

}


MatrixAdapter::~MatrixAdapter() {
	clear();
}


void MatrixAdapter::clear() {
	delete m_sparseMatrixCSR;
	m_sparseMatrixCSR = nullptr;
	delete m_denseMatrix;
	m_denseMatrix = nullptr;
}
