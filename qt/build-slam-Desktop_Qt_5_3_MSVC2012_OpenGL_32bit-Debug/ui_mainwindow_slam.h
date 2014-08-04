/********************************************************************************
** Form generated from reading UI file 'mainwindow_slam.ui'
**
** Created by: Qt User Interface Compiler version 5.3.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_SLAM_H
#define UI_MAINWINDOW_SLAM_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow_slam
{
public:
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QWidget *centralWidget;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow_slam)
    {
        if (MainWindow_slam->objectName().isEmpty())
            MainWindow_slam->setObjectName(QStringLiteral("MainWindow_slam"));
        MainWindow_slam->resize(400, 300);
        menuBar = new QMenuBar(MainWindow_slam);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        MainWindow_slam->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow_slam);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow_slam->addToolBar(mainToolBar);
        centralWidget = new QWidget(MainWindow_slam);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        MainWindow_slam->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(MainWindow_slam);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow_slam->setStatusBar(statusBar);

        retranslateUi(MainWindow_slam);

        QMetaObject::connectSlotsByName(MainWindow_slam);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow_slam)
    {
        MainWindow_slam->setWindowTitle(QApplication::translate("MainWindow_slam", "MainWindow_slam", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow_slam: public Ui_MainWindow_slam {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_SLAM_H
