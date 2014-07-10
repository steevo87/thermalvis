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
