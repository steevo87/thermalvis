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

    void on_output8bit_toggled(bool checked);

    void on_output16bit_toggled(bool checked);

    void on_outputColor_toggled(bool checked);

    void on_autoTemperature_toggled(bool checked);

    void on_undistortImages_toggled(bool checked);

    void on_normMode_currentIndexChanged(int index);

    void on_mapCode_currentIndexChanged(int index);

    void on_showExtremeColors_toggled(bool checked);

    void on_desiredDegreesPerGraylevel_textChanged(const QString &arg1);

private:
    Ui::MainWindow_streamer *ui;

    // Parameters
#ifdef _INCLUDE_INTERFACING_
	streamerRealtimeData *realtimeParameters;
#endif
};

#endif // MAINWINDOW_STREAMER_H
