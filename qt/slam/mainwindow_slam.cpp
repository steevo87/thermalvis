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
