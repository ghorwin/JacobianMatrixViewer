#ifndef MAINDIALOG_H
#define MAINDIALOG_H

#include <QDialog>

namespace Ui {
	class MainDialog;
}

namespace IBKMK {
	class SparseMatrixCSR;
//	class SparseMatrixEID;
	class DenseMatrix;
//	class BandMatrix;
}

class MainDialog : public QDialog {
	Q_OBJECT

public:
	explicit MainDialog(QWidget *parent = nullptr);
	~MainDialog();

private slots:
	void on_pushButtonReadMatrix_clicked();

private:
	Ui::MainDialog *ui;

	IBKMK::SparseMatrixCSR		*m_sparseMatrixCSR;
//	IBKMK::SparseMatrixEID		*m_sparseMatrixEID;
	IBKMK::DenseMatrix			*m_denseMatrix;

};

#endif // MAINDIALOG_H
