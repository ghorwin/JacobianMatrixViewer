#include "MainDialog.h"
#include "ui_MainDialog.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>

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

	unsigned long		m_linesRead = 0;

private:
	IBKMK::DenseMatrix	*m_matrix = nullptr;
};


MainDialog::MainDialog(QWidget *parent) :
	QDialog(parent, Qt::Window | Qt::CustomizeWindowHint | Qt::WindowMinMaxButtonsHint | Qt::WindowSystemMenuHint | Qt::WindowCloseButtonHint),
	ui(new Ui::MainDialog)
{
	ui->setupUi(this);
	ui->labelPixmap->setText(QString());

	m_tableModel = new MatrixTableModel(this);

	m_differenceMatrix.m_first = &m_mainMatrix;
	m_differenceMatrix.m_second = &m_otherMatrix;

	ui->tableView->setModel(m_tableModel);
	ui->tableView->verticalHeader()->setDefaultSectionSize(12);
	ui->tableView->horizontalHeader()->setDefaultSectionSize(55);
}


MainDialog::~MainDialog() {
	delete ui;
}


void MainDialog::on_pushButtonReadMatrix_clicked() {
	QString openFile = QFileDialog::getOpenFileName(this, tr("Select matrix file"),
													ui->lineEditMatrixFile->text(), tr("Matrix files (*.txt *.bin);;All files (*)"),
													nullptr, QFileDialog::DontUseNativeDialog);
	if (openFile.isEmpty())
		return;

	readMatrix(m_mainMatrix, openFile);
	if (m_mainMatrix.dimension() != 0)
		ui->lineEditMatrixFile->setText(openFile);
	else
		ui->lineEditMatrixFile->clear();

	// when there is no other matrix yet (i.e. first matrix is read), adjust the pixel size to fit the matrix to view
	if (ui->lineEditOtherMatrixFile->text().isEmpty()) {
		if (m_mainMatrix.dimension() > 0) {
			int pixelSize = (int)std::floor(600/m_mainMatrix.dimension());
			pixelSize =  std::min(20, pixelSize);
			ui->spinBoxPixel->setValue(pixelSize);
		}

	}
	updateView();
}


void MainDialog::on_pushButtonCompareWithOther_clicked() {
	QString initialFile = ui->lineEditOtherMatrixFile->text();
	if (initialFile.isEmpty())
		initialFile = ui->lineEditMatrixFile->text();
	QString openFile = QFileDialog::getOpenFileName(this, tr("Select matrix file"),
													initialFile, tr("Matrix files (*.txt *.bin);;All files (*)"),
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

			// read size of data storage in file
			uint64_t matSize;
			if (!in.read((char*)&matSize, sizeof(uint64_t)))
				throw IBK::Exception("Error reading matrix size from file.", FUNC_ID);
			// reserve memory
			std::string smem(matSize, ' ');
			// read matrix data
			if (!in.read(&smem[0], (std::streamsize)matSize))
				throw IBK::Exception("Error reading matrix data from file.", FUNC_ID);
			in.close();

			// determine matrix type
			void * dataPtr = (void*)&smem[0];
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
									 .arg((unsigned int)lineProcessor.m_linesRead).arg(storage.m_denseMatrix->n()), FUNC_ID);
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
			m_tableModel->setMatrixAdapter(nullptr);
		else
			m_tableModel->setMatrixAdapter(&m_mainMatrix);
	}
	else if (ui->radioButtonSecond->isChecked()) {
		if (m_otherMatrix.dimension() == 0)
			m_tableModel->setMatrixAdapter(nullptr);
		else
			m_tableModel->setMatrixAdapter(&m_otherMatrix);
	}
	else {
		if (m_differenceMatrix.dimension() == 0)
			m_tableModel->setMatrixAdapter(nullptr);
		else
			m_tableModel->setMatrixAdapter(&m_differenceMatrix);
	}

	// now generate the preview-pixmap
	ui->labelPixmap->setPixmap(generatePixmap());
	ui->labelPixmap->setMinimumSize( ui->labelPixmap->pixmap()->size());
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
			m_matrix->resize((unsigned int)vals.size());
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
					.arg((unsigned int)m_linesRead+1).arg(line), FUNC_ID);
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


QPixmap MainDialog::generatePixmap() const {
	// compute size in pixels
	int n = m_tableModel->rowCount(QModelIndex());
	if (n == 0)
		return QPixmap();

	int cellPixelSize = ui->spinBoxPixel->value();
	int pixSize = n*cellPixelSize;
	int borderLinePixelWidth = 1;
	QColor backgroundColor(Qt::white);

	// create pixmap with desired sizes but add pixels for boundary
	QPixmap pixmap(pixSize + 2*borderLinePixelWidth, pixSize + 2*borderLinePixelWidth);

	// fill pixmap with transparent color
	pixmap.fill( backgroundColor );

	QPainter p(&pixmap);

	// draw pixels
	for (int i=0; i<n; ++i) {
		for (int j=0; j<n; ++j) {
			// get state of cell
			AbstractMatrixAdapter::CellState s = m_tableModel->matrixAdapter()->state((unsigned int)i, (unsigned int)j);
			if (s == AbstractMatrixAdapter::CS_Unused)
				continue;
			QColor cellColor;
			switch (s) {
				case AbstractMatrixAdapter::CS_Used:
					cellColor = Qt::black;
				break;
				case AbstractMatrixAdapter::CS_DifferentByUsage:
					cellColor = QColor("#E50000");
				break;
				case AbstractMatrixAdapter::CS_MayBeDifferentByUsage:
					cellColor = QColor("#FFE0AD");
				break;
				case AbstractMatrixAdapter::CS_DifferentByValue:
					cellColor = QColor("#65C3FF");
				break;
				case AbstractMatrixAdapter::CS_SlightlyDifferentByValue:
					cellColor = QColor("#E6ECFF");
				break;
				case AbstractMatrixAdapter::CS_Zero :
					cellColor = backgroundColor;
				break;
				case AbstractMatrixAdapter::CS_Unused: ; // just to make compiler happy
			}
//			double val = m_tableModel->matrixAdapter()->value(i,j);
			p.fillRect(borderLinePixelWidth + j*cellPixelSize, borderLinePixelWidth + i*cellPixelSize,
					   cellPixelSize, cellPixelSize, cellColor);
		}
	}

	// draw boundary
	if (borderLinePixelWidth != 0) {
		QPen borderPen(Qt::black);
		borderPen.setWidth(borderLinePixelWidth);

		p.setPen( borderPen );
		p.drawRect(0, 0, pixSize+borderLinePixelWidth, pixSize+borderLinePixelWidth);
	}

	return pixmap;
}


void MainDialog::on_spinBoxPixel_valueChanged(int) {
	updateView();
}


void MainDialog::on_pushButtonExportImage_clicked() {
	if (m_tableModel->rowCount(QModelIndex()) == 0) {
		QMessageBox::critical(this, QString(), tr("No image to export. Load and display a matrix first!"));
		return;
	}

	QString openFile = QFileDialog::getSaveFileName(this, tr("Select image file"),
													QString(), tr("All files (*);;Bitmap file (*.png);;PDF (*.pdf);;SVG (*.svg)"),
													nullptr, QFileDialog::DontUseNativeDialog);
	if (openFile.isEmpty())
		return;

	QString ext = QFileInfo(openFile).suffix();
	if (ext.isEmpty()) {
		openFile += ".png";
		if (QFile(openFile).exists()) {
			if (QMessageBox::question(this, QString(), tr("File '%1' exists already. Overwrite?").arg(openFile), QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes)
				return;
		}
		ext = "png";
	}
	else
		ext = ext.toLower();


	// extension
	if (ext == "png") {
		// bitmap export

		if (!ui->labelPixmap->pixmap()->save(openFile)) {
			QMessageBox::critical(this, QString(), tr("Error save bitmap file '%1'.").arg(openFile));
			return;
		}
	}
	else {
		QMessageBox::critical(this, QString(), tr("Sorry, pdf and svg export is not implemented, yet."));
		return;
	}
}


void MainDialog::on_toolButtonReloadFirstMatrix_clicked() {
	QString openFile = ui->lineEditMatrixFile->text();
	if (openFile.isEmpty()) return;
	readMatrix(m_mainMatrix, openFile);
	if (m_mainMatrix.dimension() == 0)
		QMessageBox::critical(this, QString(), tr("Cannot read file '%1'").arg(openFile));
	updateView();
}


void MainDialog::on_toolButtonReloadSecondMatrix_clicked() {
	QString openFile = ui->lineEditOtherMatrixFile->text();
	if (openFile.isEmpty()) return;
	readMatrix(m_otherMatrix, openFile);
	if (m_otherMatrix.dimension() == 0)
		QMessageBox::critical(this, QString(), tr("Cannot read file '%1'").arg(openFile));
	updateView();
}
