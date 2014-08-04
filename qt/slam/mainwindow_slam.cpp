#include "mainwindow_slam.h"
#include "ui_mainwindow_slam.h"

MainWindow_slam::MainWindow_slam(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow_slam)
{
    ui->setupUi(this);
}

MainWindow_slam::~MainWindow_slam()
{
    delete ui;
}

#ifdef _INCLUDE_INTERFACING_
void MainWindow_slam::linkRealtimeVariables(slamRealtimeData* sourceData) {
    realtimeParameters = sourceData;

    //ui->debugMode->setChecked(realtimeParameters->debugMode);
    //ui->verboseMode->setChecked(realtimeParameters->verboseMode);

}
#endif
