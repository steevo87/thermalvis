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
    ui->output8bit->setChecked(realtimeParameters->output8bit);
	ui->output16bit->setChecked(realtimeParameters->output16bit);
	ui->outputColor->setChecked(realtimeParameters->outputColor);
    ui->autoTemperature->setChecked(realtimeParameters->autoTemperature);
    ui->undistortImages->setChecked(realtimeParameters->undistortImages);
    ui->showExtremeColors->setChecked(realtimeParameters->showExtremeColors);

    ui->normMode->setCurrentIndex(realtimeParameters->normMode);
    ui->mapCode->setCurrentIndex(realtimeParameters->mapCode);

    ui->desiredDegreesPerGraylevel->setText(QString::number(realtimeParameters->desiredDegreesPerGraylevel));
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

void MainWindow_streamer::on_output8bit_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->output8bit = checked;
#else
    (void)checked;
#endif
}

void MainWindow_streamer::on_output16bit_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->output16bit = checked;
#else
    (void)checked;
#endif
}

void MainWindow_streamer::on_outputColor_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->outputColor = checked;
#else
    (void)checked;
#endif
}

void MainWindow_streamer::on_autoTemperature_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->autoTemperature = checked;
#else
    (void)checked;
#endif
}

void MainWindow_streamer::on_undistortImages_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->undistortImages = checked;
#else
    (void)checked;
#endif
}

void MainWindow_streamer::on_showExtremeColors_toggled(bool checked)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->showExtremeColors = checked;
#else
    (void)checked;
#endif
}

void MainWindow_streamer::on_normMode_currentIndexChanged(int index)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->normMode = index;
#else
    (void)index;
#endif
}

void MainWindow_streamer::on_mapCode_currentIndexChanged(int index)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->mapCode = index;
#else
    (void)index;
#endif
}



void MainWindow_streamer::on_desiredDegreesPerGraylevel_textChanged(const QString &arg1)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->desiredDegreesPerGraylevel = arg1.toDouble();
#else
    (void)arg1;
#endif
}
