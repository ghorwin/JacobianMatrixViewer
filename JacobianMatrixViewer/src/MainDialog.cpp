#include "MainDialog.h"
#include "ui_MainDialog.h"

#include <QFileDialog>
#include <QMessageBox>

#include <fstream>

#include <IBK_FileReader.h>
#include <IBK_StringUtils.h>

#include <IBKMK_SparseMatrixCSR.h>
#include <IBKMK_DenseMatrix.h>
#include "IBKMK_common_defines.h"

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
	IBKMK::DenseMatrix	*m_matrix;
};


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
	FUNCID(MainDialog::on_pushButtonReadMatrix_clicked);

	QString openFile = QFileDialog::getOpenFileName(this, tr("Select matrix file"),
													QString(), tr("Plain text table (*.txt *.tsv);;Binary format (*.bin);;All files (*)"),
													nullptr, QFileDialog::DontUseNativeDialog);
	if (openFile.isEmpty())
		return;

	delete m_sparseMatrixCSR;
	m_sparseMatrixCSR = nullptr;
	delete m_denseMatrix;
	m_denseMatrix = nullptr;
	updateView();

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

			// we first parse the matrix with the file reader

			FileReaderDataProcessorValueMatrix lineProcessor(*m_denseMatrix);
			IBK::FileReader::readAll( IBK::Path(openFile.toStdString()), &lineProcessor, std::vector<std::string>(), 0, nullptr);
			// check that entire matrix was read
			if (lineProcessor.m_linesRead != m_denseMatrix->n())
				throw IBK::Exception(IBK::FormatString("Incomplete matrix in file, only got %1 rows, but matrix dimension is %2.")
									 .arg(lineProcessor.m_linesRead).arg(m_denseMatrix->n()), FUNC_ID);
		}
	}
	catch (IBK::Exception & ex) {
		ex.writeMsgStackToError();
		QMessageBox::critical(this, QString(), tr("Error reading file '%1', see console output for details.").arg(openFile));
		return;
	}
	updateView();
}


void MainDialog::updateView() {
	// we first populate the table


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
