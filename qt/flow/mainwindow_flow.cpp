#include "mainwindow_flow.h"
#include "ui_mainwindow_flow.h"

MainWindow_flow::MainWindow_flow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow_flow)
{
    ui->setupUi(this);
}

MainWindow_flow::~MainWindow_flow()
{
    delete ui;
}

#ifdef _INCLUDE_INTERFACING_
void MainWindow_flow::linkRealtimeVariables(flowRealtimeData* sourceData) {
	realtimeParameters = sourceData; 
	
	ui->debugMode->setChecked(realtimeParameters->debugMode);
	ui->verboseMode->setChecked(realtimeParameters->verboseMode);

};
#endif

void MainWindow_flow::on_debugMode_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->debugMode = checked;
#else
    (void)checked;
#endif
}

void MainWindow_flow::on_verboseMode_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->verboseMode = checked;
#else
    (void)checked;
#endif
}
