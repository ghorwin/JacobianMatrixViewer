#ifndef MATRIXTABLEMODEL_H
#define MATRIXTABLEMODEL_H

#include <QAbstractTableModel>

class AbstractMatrixAdapter;

class MatrixTableModel : public QAbstractTableModel {
public:
	MatrixTableModel(QObject *parent = Q_NULLPTR) :
		QAbstractTableModel(parent)
	{}

	void setData(const AbstractMatrixAdapter * adapter);

	QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
	int rowCount(const QModelIndex & parent) const override;
	int columnCount(const QModelIndex & parent) const override;
	QVariant data(const QModelIndex & index, int role) const override;

private:
	const AbstractMatrixAdapter * m_matrixAdapter = nullptr;

};


#endif // MATRIXTABLEMODEL_H
