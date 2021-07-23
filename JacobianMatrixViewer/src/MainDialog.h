#ifndef MAINDIALOG_H
#define MAINDIALOG_H

#include <QDialog>

#include "MatrixAdapter.h"

namespace Ui {
	class MainDialog;
}

class MatrixTableModel;

class MainDialog : public QDialog {
	Q_OBJECT

public:
	explicit MainDialog(QWidget *parent = nullptr);
	~MainDialog();

private slots:
	void on_pushButtonReadMatrix_clicked();

	void on_pushButtonCompareWithOther_clicked();

	void on_radioButtonFirst_toggled(bool checked);

	void on_radioButtonSecond_toggled(bool checked);

	void on_radioButtonDifference_toggled(bool checked);

	void on_spinBoxPixel_valueChanged(int arg1);

	void on_pushButtonExportImage_clicked();

	void on_toolButtonReloadFirstMatrix_clicked();

	void on_toolButtonReloadSecondMatrix_clicked();

private:
	void readMatrix(SingleMatrixAdapter & storage, const QString & openFile);
	void updateView();

	QPixmap generatePixmap() const;

	Ui::MainDialog *ui;

	MatrixTableModel			*m_tableModel = nullptr;

	SingleMatrixAdapter			m_mainMatrix;
	SingleMatrixAdapter			m_otherMatrix;

	ComparisonMatrixAdapter		m_differenceMatrix;
};

#endif // MAINDIALOG_H
