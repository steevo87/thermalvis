#include "mainwindow_calibrator.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow_calibrator w;
    w.show();

    return a.exec();
}
