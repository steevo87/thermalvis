#include "mainwindow_calibrator.h"
#include "ui_mainwindow_calibrator.h"

MainWindow_calibrator::MainWindow_calibrator(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow_calibrator)
{
    ui->setupUi(this);
}

MainWindow_calibrator::~MainWindow_calibrator()
{
    delete ui;
}

#ifdef _INCLUDE_INTERFACING_
void MainWindow_calibrator::linkRealtimeVariables(calibratorRealtimeData* sourceData) {
	realtimeParameters = sourceData; 
	
	ui->debugMode->setChecked(realtimeParameters->debugMode);
	ui->verboseMode->setChecked(realtimeParameters->verboseMode);

}
#endif

void MainWindow_calibrator::on_debugMode_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->debugMode = checked;
#else
    (void)checked;
#endif
}

void MainWindow_calibrator::on_verboseMode_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->verboseMode = checked;
#else
    (void)checked;
#endif
}
