#ifndef MAINWINDOW_CALIBRATOR_H
#define MAINWINDOW_CALIBRATOR_H

#include <QMainWindow>

#ifdef _INCLUDE_INTERFACING_
#include "../../include/calibrator/calibrator_config.hpp"
#endif
namespace Ui {
class MainWindow_calibrator;
}

class MainWindow_calibrator : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow_calibrator(QWidget *parent = 0);
    ~MainWindow_calibrator();
#ifdef _INCLUDE_INTERFACING_
    void linkRealtimeVariables(calibratorRealtimeData* sourceData);
#endif

private slots:
    void on_debugMode_toggled(bool checked);

    void on_verboseMode_toggled(bool checked);

private:
    Ui::MainWindow_calibrator *ui;

    // Parameters
#ifdef _INCLUDE_INTERFACING_
    calibratorRealtimeData *realtimeParameters;
#endif
};

#endif
