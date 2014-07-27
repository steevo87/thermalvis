#ifndef MAINWINDOW_FLOW_H
#define MAINWINDOW_FLOW_H

#include <QMainWindow>

#ifdef _INCLUDE_INTERFACING_
#include "../../include/flow/flow_config.hpp"
#endif
namespace Ui {
class MainWindow_flow;
}

class MainWindow_flow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow_flow(QWidget *parent = 0);
    ~MainWindow_flow();
#ifdef _INCLUDE_INTERFACING_
    void linkRealtimeVariables(flowRealtimeData* sourceData);
#endif

private slots:
    void on_debugMode_toggled(bool checked);

    void on_verboseMode_toggled(bool checked);

    void on_showTrackHistory_toggled(bool checked);

    void on_adaptiveWindow_toggled(bool checked);

    void on_velocityPrediction_toggled(bool checked);

    void on_attemptHistoricalRecovery_toggled(bool checked);

    void on_autoTrackManagement_toggled(bool checked);

    void on_attemptMatching_toggled(bool checked);

    void on_detectEveryFrame_toggled(bool checked);

    void on_maxFeatures_returnPressed();

    void on_minFeatures_returnPressed();

    void on_drawingHistory_returnPressed();

    void on_matchingMode_currentIndexChanged(int index);

    void on_maxFrac_returnPressed();

    void on_minSeparation_returnPressed();

    void on_maxVelocity_returnPressed();

    void on_newFeaturesPeriod_returnPressed();

    void on_delayTimeout_returnPressed();

    void on_sensitivity_1_returnPressed();

    void on_sensitivity_2_returnPressed();

    void on_sensitivity_3_returnPressed();

    void on_multiplier_1_returnPressed();

    void on_multiplier_2_returnPressed();

    void on_detector_1_currentIndexChanged(int index);

    void on_detector_2_currentIndexChanged(int index);

    void on_detector_3_currentIndexChanged(int index);

private:
    Ui::MainWindow_flow *ui;

    // Parameters
#ifdef _INCLUDE_INTERFACING_
    flowRealtimeData *realtimeParameters;
#endif
};

#endif
