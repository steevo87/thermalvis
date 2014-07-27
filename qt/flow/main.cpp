#include "mainwindow_flow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow_flow w;
    w.show();

    return a.exec();
}
