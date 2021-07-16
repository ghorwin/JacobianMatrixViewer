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
	void updateView();

	Ui::MainDialog *ui;

	IBKMK::SparseMatrixCSR		*m_sparseMatrixCSR = nullptr;
//	IBKMK::SparseMatrixEID		*m_sparseMatrixEID = nullptr;
	IBKMK::DenseMatrix			*m_denseMatrix = nullptr;

};

#endif // MAINDIALOG_H
