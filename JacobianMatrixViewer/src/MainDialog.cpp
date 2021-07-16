#include "MainDialog.h"
#include "ui_MainDialog.h"

#include <QFileDialog>
#include <QMessageBox>

#include <fstream>

#include <IBK_FileReader.h>
#include <IBK_StringUtils.h>

#include <IBKMK_SparseMatrixCSR.h>
#include <IBKMK_DenseMatrix.h>
#include <IBKMK_common_defines.h>

#include "MatrixTableModel.h"

class FileReaderDataProcessorValueMatrix : public IBK::AbstractFileReaderDataProcessor {
public:
	/*! Standard constructor. Takes a reference of an external line vector.
		\param lines Line vector.
	*/
	FileReaderDataProcessorValueMatrix(IBKMK::DenseMatrix & matrix) : m_matrix(&matrix) {}

	/*! Adds a line to the line vector. Error is only possible in case of unsufficent memory.
		\param line Line for processing.
	*/
	virtual void processLine(const std::string& line) override;
	virtual int lineCount() const override { return (int)m_matrix->n(); }

	unsigned int		m_linesRead = 0;

private:
	IBKMK::DenseMatrix	*m_matrix = nullptr;
};


MainDialog::MainDialog(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::MainDialog)
{
	ui->setupUi(this);

	m_tableModel = new MatrixTableModel(this);

	ui->tableView->setModel(m_tableModel);
	ui->tableView->verticalHeader()->setDefaultSectionSize(12);
	ui->tableView->horizontalHeader()->setDefaultSectionSize(55);
}


MainDialog::~MainDialog() {
	delete ui;
}


void MainDialog::on_pushButtonReadMatrix_clicked() {
	QString openFile = QFileDialog::getOpenFileName(this, tr("Select matrix file"),
													QString(), tr("Plain text table (*.txt *.tsv);;Binary format (*.bin);;All files (*)"),
													nullptr, QFileDialog::DontUseNativeDialog);
	if (openFile.isEmpty())
		return;

	readMatrix(m_mainMatrix, openFile);
	if (m_mainMatrix.dimension() != 0)
		ui->lineEditMatrixFile->setText(openFile);
	else
		ui->lineEditMatrixFile->clear();
	updateView();
}


void MainDialog::on_pushButtonCompareWithOther_clicked() {
	QString openFile = QFileDialog::getOpenFileName(this, tr("Select matrix file"),
													QString(), tr("Plain text table (*.txt *.tsv);;Binary format (*.bin);;All files (*)"),
													nullptr, QFileDialog::DontUseNativeDialog);
	if (openFile.isEmpty())
		return;

	readMatrix(m_otherMatrix, openFile);
	if (m_otherMatrix.dimension() != 0) {
		ui->lineEditOtherMatrixFile->setText(openFile);
		ui->radioButtonSecond->setChecked(true);
	}
	else
		ui->lineEditOtherMatrixFile->clear();
	updateView();
}


void MainDialog::readMatrix(SingleMatrixAdapter & storage, const QString & openFile) {
	FUNCID(MainDialog::readMatrix);

	storage.clear();

	try {
		// if binary mode, ask user about matrix type
		if (openFile.toLower().endsWith("bin")) {
			// read content of matrix into memory
			std::ifstream in(openFile.toStdString(), std::ios_base::binary);
			if (!in)
				throw IBK::Exception("File io error.", FUNC_ID);

			std::vector<unsigned char> buffer(std::istreambuf_iterator<char>(in), {});

			// determine matrix type
			void * dataPtr = buffer.data();
			char matType = *(char*)dataPtr;

			switch (matType) {
				case MT_DenseMatrix :  {
					storage.m_denseMatrix = new IBKMK::DenseMatrix;
					storage.m_denseMatrix->recreate(dataPtr);
				} break;

				case MT_SparseMatrixCSR : {
					storage.m_sparseMatrixCSR = new IBKMK::SparseMatrixCSR;
					storage.m_sparseMatrixCSR->recreate(dataPtr);
				} break;
			}
		}
		else {
			// we only know dense matrix text files, but some with [] and some without
			storage.m_denseMatrix = new IBKMK::DenseMatrix;

			// we first parse the matrix with the file reader

			FileReaderDataProcessorValueMatrix lineProcessor(*storage.m_denseMatrix);
			IBK::FileReader::readAll( IBK::Path(openFile.toStdString()), &lineProcessor, std::vector<std::string>(), 0, nullptr);
			// check that entire matrix was read
			if (lineProcessor.m_linesRead != storage.m_denseMatrix->n())
				throw IBK::Exception(IBK::FormatString("Incomplete matrix in file, only got %1 rows, but matrix dimension is %2.")
									 .arg(lineProcessor.m_linesRead).arg(storage.m_denseMatrix->n()), FUNC_ID);
		}
	}
	catch (IBK::Exception & ex) {
		ex.writeMsgStackToError();
		QMessageBox::critical(this, QString(), tr("Error reading file '%1', see console output for details.").arg(openFile));
		return;
	}
}


void MainDialog::updateView() {
	// we first populate the table
	if (ui->radioButtonFirst->isChecked()) {
		if (m_mainMatrix.dimension() == 0)
			m_tableModel->setData(nullptr);
		else
			m_tableModel->setData(&m_mainMatrix);
	}
	else if (ui->radioButtonSecond->isChecked()) {
		if (m_otherMatrix.dimension() == 0)
			m_tableModel->setData(nullptr);
		else
			m_tableModel->setData(&m_otherMatrix);
	}
}


void FileReaderDataProcessorValueMatrix::processLine(const std::string& line) {
	FUNCID(FileReaderDataProcessorValueMatrix::processLine);

	// remove leading/trailing [] from line
	std::string modifiedLine = line;
	IBK::trim(modifiedLine, " \t[]\r");
	if (modifiedLine.empty())
		return; // skip empty lines

	std::vector<double> vals;
	try {
		IBK::string2valueVector(modifiedLine, vals);
		// if this is the first line, we resize the target matrix accordingly
		if (m_linesRead == 0)
			m_matrix->resize(vals.size());
		else {
			// otherwise check that dimensions match
			if (m_matrix->n() != vals.size())
				throw IBK::Exception( IBK::FormatString("Mismatching dimensions. Expected %1, got %2.").arg(m_matrix->n()).arg(vals.size()), FUNC_ID);
		}

		// store data; we have column-major order, so we have to do this value-by-value
		for (unsigned int i=0; i<vals.size(); ++i)
			(*m_matrix)(m_linesRead, i) = vals[i];
		++m_linesRead; // increase line count
	}
	catch (IBK::Exception & ex) {
		throw IBK::Exception(ex, IBK::FormatString("Error parsing line #%1 with content '%2'.")
					.arg(m_linesRead+1).arg(line), FUNC_ID);
	}
}


void MainDialog::on_radioButtonFirst_toggled(bool checked) {
	if (checked)
		updateView();
}

void MainDialog::on_radioButtonSecond_toggled(bool checked) {
	if (checked)
		updateView();
}

void MainDialog::on_radioButtonDifference_toggled(bool checked) {
	if (checked)
		updateView();
}
