#include "MatrixTableModel.h"

#include "MatrixAdapter.h"

void MatrixTableModel::setData(const MatrixAdapter * adapter) {
	beginResetModel();
	m_matrixAdapter = adapter;
	endResetModel();
}

int MatrixTableModel::rowCount(const QModelIndex & parent) const {
	if (m_matrixAdapter->)
}

int MatrixTableModel::columnCount(const QModelIndex & parent) const
{
}

QVariant MatrixTableModel::data(const QModelIndex & index, int role) const
{
}
