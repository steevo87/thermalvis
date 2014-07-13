#ifndef MAINWINDOW_STREAMER_H
#define MAINWINDOW_STREAMER_H

#include <QMainWindow>

#ifdef _INCLUDE_INTERFACING_
#include "../../include/input_stream_config.hpp"
#endif
namespace Ui {
class MainWindow_streamer;
}

class MainWindow_streamer : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow_streamer(QWidget *parent = 0);
    ~MainWindow_streamer();
#ifdef _INCLUDE_INTERFACING_
	void linkRealtimeVariables(streamerRealtimeData* sourceData);
#endif

private slots:
    void on_debugMode_toggled(bool checked);

    void on_verboseMode_toggled(bool checked);

private:
    Ui::MainWindow_streamer *ui;

    // Parameters
#ifdef _INCLUDE_INTERFACING_
	streamerRealtimeData *realtimeParameters;
#endif
};

#endif // MAINWINDOW_STREAMER_H
