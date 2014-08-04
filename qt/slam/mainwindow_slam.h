#ifndef MAINWINDOW_SLAM_H
#define MAINWINDOW_SLAM_H

#include <QMainWindow>

namespace Ui {
class MainWindow_slam;
}

class MainWindow_slam : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow_slam(QWidget *parent = 0);
    ~MainWindow_slam();

private:
    Ui::MainWindow_slam *ui;
};

#endif // MAINWINDOW_SLAM_H
