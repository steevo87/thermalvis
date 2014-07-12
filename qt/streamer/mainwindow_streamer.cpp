#include "mainwindow_streamer.h"
#include "ui_mainwindow_streamer.h"

MainWindow_streamer::MainWindow_streamer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow_streamer),
	debugMode(false),
	verboseMode(false)
{
    ui->setupUi(this);
}

MainWindow_streamer::~MainWindow_streamer()
{
    delete ui;
}

void MainWindow_streamer::on_debugMode_toggled(bool checked)
{
    debugMode = checked;
}

void MainWindow_streamer::on_verboseMode_toggled(bool checked)
{
    verboseMode = checked;
}
