#ifndef MAINWINDOW_STREAMER_H
#define MAINWINDOW_STREAMER_H

#include <QMainWindow>

namespace Ui {
class MainWindow_streamer;
}

class MainWindow_streamer : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow_streamer(QWidget *parent = 0);
    ~MainWindow_streamer();

private:
    Ui::MainWindow_streamer *ui;
};

#endif // MAINWINDOW_STREAMER_H
