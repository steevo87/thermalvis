#ifndef MAINWINDOW_SLAM_H
#define MAINWINDOW_SLAM_H

#include <QMainWindow>

#ifdef _INCLUDE_INTERFACING_
#include "../../include/slam/slam_config.hpp"
#endif

namespace Ui {
class MainWindow_slam;
}

class MainWindow_slam : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow_slam(QWidget *parent = 0);
    ~MainWindow_slam();
#ifdef _INCLUDE_INTERFACING_
    void linkRealtimeVariables(slamRealtimeData* sourceData);
#endif

private:
    Ui::MainWindow_slam *ui;

#ifdef _INCLUDE_INTERFACING_
    slamRealtimeData *realtimeParameters;
#endif

};

#endif // MAINWINDOW_SLAM_H
