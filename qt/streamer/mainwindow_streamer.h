#ifndef MAINWINDOW_STREAMER_H
#define MAINWINDOW_STREAMER_H

#include <QMainWindow>

#ifdef _INCLUDE_INTERFACING_
#include "../../include/streamer/streamer_config.hpp"
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

	void on_denoisingMethod_currentIndexChanged(int index);

    void on_inputDatatype_currentIndexChanged(int index);

    void on_detectorMode_currentIndexChanged(int index);

    void on_usbMode_currentIndexChanged(int index);

    void on_maxReadAttempts_returnPressed();

    void on_desiredDegreesPerGraylevel_returnPressed();

    void on_maxNucInterval_returnPressed();

    void on_zeroDegreesOffset_returnPressed();

    void on_degreesPerGraylevel_returnPressed();

    void on_framerate_returnPressed();

    void on_threshFactor_returnPressed();

    void on_normFactor_returnPressed();

    void on_fusionFactor_returnPressed();

    void on_serialPollingRate_returnPressed();

    void on_maxNucThreshold_returnPressed();

    void on_minTemp_returnPressed();

    void on_maxTemp_returnPressed();

private:
    Ui::MainWindow_streamer *ui;

    // Parameters
#ifdef _INCLUDE_INTERFACING_
	streamerRealtimeData *realtimeParameters;
#endif
};

#endif // MAINWINDOW_STREAMER_H
