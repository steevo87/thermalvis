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

	bool getDebugMode() { return debugMode; }
	bool getVerboseMode() { return verboseMode; }

private slots:
    void on_debugMode_toggled(bool checked);

    void on_verboseMode_toggled(bool checked);

private:
    Ui::MainWindow_streamer *ui;

    // Parameters
    bool debugMode;
    bool verboseMode;
};

#endif // MAINWINDOW_STREAMER_H
