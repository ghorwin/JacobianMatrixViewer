#include "MainDialog.h"
#include "ui_MainDialog.h"

#include <QFileDialog>
#include <QMessageBox>

#include <fstream>

#include <IBKMK_SparseMatrixCSR.h>
#include <IBKMK_DenseMatrix.h>
#include "IBKMK_common_defines.h"

MainDialog::MainDialog(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::MainDialog)
{
	ui->setupUi(this);
}


MainDialog::~MainDialog() {
	delete ui;

	delete m_sparseMatrixCSR;
	m_sparseMatrixCSR = nullptr;
	delete m_denseMatrix;
	m_denseMatrix = nullptr;
}


void MainDialog::on_pushButtonReadMatrix_clicked() {
	QString openFile = QFileDialog::getOpenFileName(this, tr("Select matrix file"),
													QString(), tr("Plain text table (*.txt *.tsv);;Binary format (*.bin);;All files (*)"),
													nullptr, QFileDialog::DontUseNativeDialog);
	if (openFile.isEmpty())
		return;

	delete m_sparseMatrixCSR;
	m_sparseMatrixCSR = nullptr;
	delete m_denseMatrix;
	m_denseMatrix = nullptr;

	// if binary mode, ask user about matrix type
	if (openFile.toLower().endsWith("bin")) {
		// read content of matrix into memory
		std::ifstream in(openFile.toStdString(), std::ios_base::binary);
		std::vector<unsigned char> buffer(std::istreambuf_iterator<char>(in), {});

		// determine matrix type
		void * dataPtr = buffer.data();
		char matType = *(char*)dataPtr;

		switch (matType) {
			case MT_DenseMatrix :  {
				m_denseMatrix = new IBKMK::DenseMatrix;
				m_denseMatrix->recreate(dataPtr);
			} break;

			case MT_SparseMatrixCSR : {
				m_sparseMatrixCSR = new IBKMK::SparseMatrixCSR;
				m_sparseMatrixCSR->recreate(dataPtr);
			} break;
		}
	}
	else {
		// we only know dense matrix text files, but some with [] and some without
		m_denseMatrix = new IBKMK::DenseMatrix;


	}


}
