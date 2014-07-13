#include "mainwindow_streamer.h"
#include "ui_mainwindow_streamer.h"

MainWindow_streamer::MainWindow_streamer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow_streamer)
{
    ui->setupUi(this);
}

MainWindow_streamer::~MainWindow_streamer()
{
    delete ui;
}

#ifdef _INCLUDE_INTERFACING_
void MainWindow_streamer::linkRealtimeVariables(streamerRealtimeData* sourceData) { 
	realtimeParameters = sourceData; 
	
	ui->debugMode->setChecked(realtimeParameters->debugMode);
	ui->verboseMode->setChecked(realtimeParameters->verboseMode);
};
#endif

void MainWindow_streamer::on_debugMode_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->debugMode = checked;
#else
    (void)checked;
#endif
}

void MainWindow_streamer::on_verboseMode_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->verboseMode = checked;
#else
    (void)checked;
#endif
}
