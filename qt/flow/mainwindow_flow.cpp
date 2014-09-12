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

    ui->matchingMode->setCurrentIndex(realtimeParameters->matchingMode);
    ui->detector_1->setCurrentIndex(realtimeParameters->detector_1);
    ui->detector_2->setCurrentIndex(realtimeParameters->detector_2);
    ui->detector_3->setCurrentIndex(realtimeParameters->detector_3);

    ui->maxFeatures->setText(QString::number(realtimeParameters->maxFeatures));
    ui->minFeatures->setText(QString::number(realtimeParameters->minFeatures));
    ui->drawingHistory->setText(QString::number(realtimeParameters->drawingHistory));
    ui->maxFrac->setText(QString::number(realtimeParameters->maxFrac));

    ui->minSeparation->setText(QString::number(realtimeParameters->minSeparation));
    ui->maxVelocity->setText(QString::number(realtimeParameters->maxVelocity));
    ui->newFeaturesPeriod->setText(QString::number(realtimeParameters->newFeaturesPeriod));
    ui->delayTimeout->setText(QString::number(realtimeParameters->delayTimeout));

    ui->sensitivity_1->setText(QString::number(realtimeParameters->sensitivity_1));
    ui->sensitivity_2->setText(QString::number(realtimeParameters->sensitivity_2));
    ui->sensitivity_3->setText(QString::number(realtimeParameters->sensitivity_3));

    ui->multiplier_1->setText(QString::number(realtimeParameters->multiplier_1));
    ui->multiplier_2->setText(QString::number(realtimeParameters->multiplier_2));

    ui->flowThreshold->setText(QString::number(realtimeParameters->flowThreshold));

}
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

void MainWindow_flow::on_maxFeatures_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->maxFeatures = ui->maxFeatures->text().toInt();
#else
    (void)0;
#endif
}

void MainWindow_flow::on_minFeatures_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->minFeatures = ui->minFeatures->text().toInt();
#else
    (void)0;
#endif
}

void MainWindow_flow::on_drawingHistory_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->drawingHistory = ui->drawingHistory->text().toInt();
#else
    (void)0;
#endif
}

void MainWindow_flow::on_matchingMode_currentIndexChanged(int index)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->matchingMode = index;
#else
    (void)index;
#endif
}

void MainWindow_flow::on_maxFrac_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->maxFrac = ui->maxFrac->text().toDouble();
#else
    (void)0;
#endif
}

void MainWindow_flow::on_minSeparation_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->minSeparation = ui->minSeparation->text().toInt();
#else
    (void)0;
#endif
}

void MainWindow_flow::on_maxVelocity_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->maxVelocity = ui->maxVelocity->text().toInt();
#else
    (void)0;
#endif
}

void MainWindow_flow::on_newFeaturesPeriod_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->newFeaturesPeriod = ui->newFeaturesPeriod->text().toInt();
#else
    (void)0;
#endif
}

void MainWindow_flow::on_delayTimeout_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->delayTimeout = ui->delayTimeout->text().toInt();
#else
    (void)0;
#endif
}

void MainWindow_flow::on_sensitivity_1_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->sensitivity_1 = ui->sensitivity_1->text().toDouble();
#else
    (void)0;
#endif
}

void MainWindow_flow::on_sensitivity_2_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->sensitivity_2 = ui->sensitivity_2->text().toDouble();
#else
    (void)0;
#endif
}

void MainWindow_flow::on_sensitivity_3_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->sensitivity_3 = ui->sensitivity_3->text().toDouble();
#else
    (void)0;
#endif
}

void MainWindow_flow::on_multiplier_1_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->multiplier_1 = ui->multiplier_1->text().toInt();
#else
    (void)0;
#endif
}

void MainWindow_flow::on_multiplier_2_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->multiplier_2 = ui->multiplier_2->text().toInt();
#else
    (void)0;
#endif
}


void MainWindow_flow::on_flowThreshold_returnPressed()
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->flowThreshold = ui->flowThreshold->text().toDouble();
#else
    (void)0;
#endif
}

void MainWindow_flow::on_detector_1_currentIndexChanged(int index)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->detector_1 = index;
#else
    (void)index;
#endif
}

void MainWindow_flow::on_detector_2_currentIndexChanged(int index)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->detector_2 = index;
#else
    (void)index;
#endif
}

void MainWindow_flow::on_detector_3_currentIndexChanged(int index)
{
#ifdef _INCLUDE_INTERFACING_
    realtimeParameters->detector_3 = index;
#else
    (void)index;
#endif
}

