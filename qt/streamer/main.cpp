#include "mainwindow_streamer.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow_streamer w;
    w.show();

    return a.exec();
}
