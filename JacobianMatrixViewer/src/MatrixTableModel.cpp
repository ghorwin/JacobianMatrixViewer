#include "MatrixTableModel.h"

#include <QColor>
#include <QFont>
#include <QSize>

#include "MatrixAdapter.h"

void MatrixTableModel::setData(const AbstractMatrixAdapter * adapter) {
	beginResetModel();
	m_matrixAdapter = adapter;
	endResetModel();
}


int MatrixTableModel::rowCount(const QModelIndex & /*parent*/) const {
	if (m_matrixAdapter == nullptr)
		return 0;
	else
		return (int)m_matrixAdapter->dimension();
}


int MatrixTableModel::columnCount(const QModelIndex & /*parent*/) const {
	if (m_matrixAdapter == nullptr)
		return 0;
	else
		return (int)m_matrixAdapter->dimension();
}


QVariant MatrixTableModel::headerData(int section, Qt::Orientation orientation, int role) const {
	if (m_matrixAdapter == nullptr)
		return QVariant();
	if (orientation == Qt::Horizontal) {
		switch (role) {
			case Qt::DisplayRole :
				return QString("%1").arg(section+1);

			case Qt::FontRole : {
				QFont f;
				f.setPointSize(8);
				return f;
			}
			case Qt::SizeHintRole :
				return QSize(20,12);
		}
	}
	else {
		switch (role) {
			case Qt::DisplayRole :
				return QString("%1").arg(section+1);
			case Qt::FontRole : {
				QFont f;
				f.setPointSize(8);
				return f;
			}
			case Qt::SizeHintRole :
				return QSize(20,12);
		}
	}
	return QVariant();
}

QVariant MatrixTableModel::data(const QModelIndex & index, int role) const {
	// we implement several roles
	if (m_matrixAdapter == nullptr)
		return QVariant();

	// displayrole is the value
	switch (role) {
		case Qt::DisplayRole : {
			AbstractMatrixAdapter::CellState cs = m_matrixAdapter->state((unsigned int)index.row(), (unsigned int)index.column());
			if (cs != AbstractMatrixAdapter::CS_Unused)
				return QString("%1").arg(m_matrixAdapter->value((unsigned int)index.row(), (unsigned int)index.column()), 5, 'g',3);
			else
				return QVariant();
		}
		case Qt::BackgroundRole :
			switch (m_matrixAdapter->state((unsigned int)index.row(), (unsigned int)index.column())) {
				case AbstractMatrixAdapter::CS_Used:
					return QColor("#B9EF8F");
				case AbstractMatrixAdapter::CS_Unused:
					return QColor(192,192,192);
				case AbstractMatrixAdapter::CS_Zero:
					return QColor(Qt::white);

				case AbstractMatrixAdapter::CS_DifferentByUsage:
					return QColor("#E50000");
				case AbstractMatrixAdapter::CS_MayBeDifferentByUsage:
					return QColor("#FFE0AD");

				case AbstractMatrixAdapter::CS_DifferentByValue:
					return QColor("#65C3FF");
				case AbstractMatrixAdapter::CS_SlightlyDifferentByValue:
					return QColor("#E6ECFF");
			}
		break;

		case Qt::FontRole : {
			QFont f;
			f.setPointSize(8);
			return f;
		}
	}
	return QVariant();
}
