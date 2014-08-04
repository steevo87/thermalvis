#include "mainwindow_slam.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow_slam w;
    w.show();

    return a.exec();
}
