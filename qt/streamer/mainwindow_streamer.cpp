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
    ui->undistortImages->setChecked(realtimeParameters->wantsToUndistort);

    ui->normMode->setCurrentIndex(realtimeParameters->normMode);
    ui->mapCode->setCurrentIndex(realtimeParameters->map);
    ui->inputDatatype->setCurrentIndex(realtimeParameters->inputDatatype);
    ui->detectorMode->setCurrentIndex(realtimeParameters->detectorMode);
    ui->usbMode->setCurrentIndex(realtimeParameters->usbMode);

    ui->maxReadAttempts->setText(QString::number(realtimeParameters->maxReadAttempts));
    ui->desiredDegreesPerGraylevel->setText(QString::number(realtimeParameters->desiredDegreesPerGraylevel));
    ui->maxNucInterval->setText(QString::number(realtimeParameters->maxNucInterval));
    ui->zeroDegreesOffset->setText(QString::number(realtimeParameters->zeroDegreesOffset));
    ui->degreesPerGraylevel->setText(QString::number(realtimeParameters->degreesPerGraylevel));
    ui->framerate->setText(QString::number(realtimeParameters->framerate));
    ui->threshFactor->setText(QString::number(realtimeParameters->threshFactor));
    ui->normFactor->setText(QString::number(realtimeParameters->normFactor));
    ui->fusionFactor->setText(QString::number(realtimeParameters->fusionFactor));
    ui->serialPollingRate->setText(QString::number(realtimeParameters->serialPollingRate));
    ui->maxNucThreshold->setText(QString::number(realtimeParameters->maxNucThreshold));
    ui->minTemp->setText(QString::number(realtimeParameters->minTemperature));
    ui->maxTemp->setText(QString::number(realtimeParameters->maxTemperature));

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
    realtimeParameters->wantsToUndistort = checked;
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
    realtimeParameters->map = index;
#else
    (void)index;
#endif
}

void MainWindow_streamer::on_inputDatatype_currentIndexChanged(int index)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->inputDatatype = index;
#else
    (void)index;
#endif
}

void MainWindow_streamer::on_detectorMode_currentIndexChanged(int index)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->detectorMode = index;
#else
    (void)index;
#endif
}

void MainWindow_streamer::on_usbMode_currentIndexChanged(int index)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->usbMode = index;
#else
    (void)index;
#endif
}

void MainWindow_streamer::on_maxReadAttempts_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->maxReadAttempts = ui->maxReadAttempts->text().toInt();
#else
    (void)0;
#endif
}

void MainWindow_streamer::on_desiredDegreesPerGraylevel_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->desiredDegreesPerGraylevel = ui->desiredDegreesPerGraylevel->text().toDouble();
#else
    (void)0;
#endif
}

void MainWindow_streamer::on_maxNucInterval_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->maxNucInterval = ui->maxNucInterval->text().toInt();
#else
    (void)0;
#endif
}

void MainWindow_streamer::on_zeroDegreesOffset_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->zeroDegreesOffset = ui->zeroDegreesOffset->text().toInt();
#else
    (void)0;
#endif
}

void MainWindow_streamer::on_degreesPerGraylevel_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->degreesPerGraylevel = ui->degreesPerGraylevel->text().toDouble();
#else
    (void)0;
#endif
}

void MainWindow_streamer::on_framerate_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->framerate = ui->framerate->text().toDouble();
#else
    (void)0;
#endif
}

void MainWindow_streamer::on_threshFactor_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->threshFactor = ui->threshFactor->text().toDouble();
#else
    (void)0;
#endif
}

void MainWindow_streamer::on_normFactor_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->normFactor = ui->normFactor->text().toDouble();
#else
    (void)0;
#endif
}

void MainWindow_streamer::on_fusionFactor_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->fusionFactor = ui->fusionFactor->text().toDouble();
#else
    (void)0;
#endif
}

void MainWindow_streamer::on_serialPollingRate_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->serialPollingRate = ui->serialPollingRate->text().toDouble();
#else
    (void)0;
#endif
}

void MainWindow_streamer::on_maxNucThreshold_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->maxNucThreshold = ui->maxNucThreshold->text().toDouble();
#else
    (void)0;
#endif
}

void MainWindow_streamer::on_minTemp_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->minTemperature = ui->minTemp->text().toDouble();
#else
    (void)0;
#endif
}

void MainWindow_streamer::on_maxTemp_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->maxTemperature = ui->maxTemp->text().toDouble();
#else
    (void)0;
#endif
}
