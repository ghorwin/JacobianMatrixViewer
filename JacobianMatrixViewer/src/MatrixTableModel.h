#ifndef MATRIXTABLEMODEL_H
#define MATRIXTABLEMODEL_H

#include <QAbstractTableModel>

class MatrixAdapter;

class MatrixTableModel : public QAbstractTableModel {
public:
	MatrixTableModel(QObject *parent = Q_NULLPTR) :
		QAbstractTableModel(parent)
	{}

	void setData(const MatrixAdapter * adapter);

	int rowCount(const QModelIndex & parent) const override;
	int columnCount(const QModelIndex & parent) const override;
	QVariant data(const QModelIndex & index, int role) const override;

private:
	const MatrixAdapter * m_matrixAdapter = nullptr;
};


#endif // MATRIXTABLEMODEL_H
