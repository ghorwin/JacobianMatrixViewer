#ifndef MAINDIALOG_H
#define MAINDIALOG_H

#include <QDialog>

#include "MatrixAdapter.h"

namespace Ui {
	class MainDialog;
}


class MainDialog : public QDialog {
	Q_OBJECT

public:
	explicit MainDialog(QWidget *parent = nullptr);
	~MainDialog();

private slots:
	void on_pushButtonReadMatrix_clicked();

private:
	void readMatrix(MatrixAdapater & storage, const QString & openFile);
	void updateView();

	Ui::MainDialog *ui;

	MatrixAdapter			m_mainMatrix;
	MatrixAdapter			m_otherMatrix;
};

#endif // MAINDIALOG_H
