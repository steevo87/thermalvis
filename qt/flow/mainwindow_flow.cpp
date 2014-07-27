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

    ui->showTrackHistory->setChecked(realtimeParameters->showTrackHistory);
    ui->adaptiveWindow->setChecked(realtimeParameters->adaptiveWindow);
    ui->velocityPrediction->setChecked(realtimeParameters->velocityPrediction);
    ui->attemptHistoricalRecovery->setChecked(realtimeParameters->attemptHistoricalRecovery);
    ui->autoTrackManagement->setChecked(realtimeParameters->autoTrackManagement);
    ui->attemptMatching->setChecked(realtimeParameters->attemptMatching);
    ui->detectEveryFrame->setChecked(realtimeParameters->detectEveryFrame);


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

void MainWindow_flow::on_showTrackHistory_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->showTrackHistory = checked;
#else
    (void)checked;
#endif
}

void MainWindow_flow::on_adaptiveWindow_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->adaptiveWindow = checked;
#else
    (void)checked;
#endif
}

void MainWindow_flow::on_velocityPrediction_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->velocityPrediction = checked;
#else
    (void)checked;
#endif
}

void MainWindow_flow::on_attemptHistoricalRecovery_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->attemptHistoricalRecovery = checked;
#else
    (void)checked;
#endif
}

void MainWindow_flow::on_autoTrackManagement_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->autoTrackManagement = checked;
#else
    (void)checked;
#endif
}

void MainWindow_flow::on_attemptMatching_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->attemptMatching = checked;
#else
    (void)checked;
#endif
}

void MainWindow_flow::on_detectEveryFrame_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->detectEveryFrame = checked;
#else
    (void)checked;
#endif
}
