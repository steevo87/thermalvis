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

private:
    Ui::MainWindow_flow *ui;

    // Parameters
#ifdef _INCLUDE_INTERFACING_
    flowRealtimeData *realtimeParameters;
#endif
};

#endif
