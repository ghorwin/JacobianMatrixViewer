#include "MainDialog.h"
#include "ui_MainDialog.h"

#include <QFileDialog>

MainDialog::MainDialog(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::MainDialog)
{
	ui->setupUi(this);
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

	// if binary file, read in binary mode


}
