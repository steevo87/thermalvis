/********************************************************************************
** Form generated from reading UI file 'mainwindow_calibrator.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_CALIBRATOR_H
#define UI_MAINWINDOW_CALIBRATOR_H

#include <QVariant>
#include <QAction>
#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QHeaderView>
#include <QMainWindow>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow_calibrator
{
public:
    QWidget *centralWidget;
    QCheckBox *debugMode;
    QCheckBox *verboseMode;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow_calibrator)
    {
        if (MainWindow_calibrator->objectName().isEmpty())
            MainWindow_calibrator->setObjectName(QString::fromUtf8("MainWindow_calibrator"));
        MainWindow_calibrator->resize(640, 320);
        centralWidget = new QWidget(MainWindow_calibrator);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        debugMode = new QCheckBox(centralWidget);
        debugMode->setObjectName(QString::fromUtf8("debugMode"));
        debugMode->setGeometry(QRect(10, 10, 81, 17));
        verboseMode = new QCheckBox(centralWidget);
        verboseMode->setObjectName(QString::fromUtf8("verboseMode"));
        verboseMode->setGeometry(QRect(10, 30, 91, 17));
        MainWindow_calibrator->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow_calibrator);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 640, 21));
        MainWindow_calibrator->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow_calibrator);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow_calibrator->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow_calibrator);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow_calibrator->setStatusBar(statusBar);

        retranslateUi(MainWindow_calibrator);

        QMetaObject::connectSlotsByName(MainWindow_calibrator);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow_calibrator)
    {
        MainWindow_calibrator->setWindowTitle(QApplication::translate("MainWindow_calibrator", "MainWindow_calibrator", 0));
        debugMode->setText(QApplication::translate("MainWindow_calibrator", "debugMode", 0));
        verboseMode->setText(QApplication::translate("MainWindow_calibrator", "verboseMode", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow_calibrator: public Ui_MainWindow_calibrator {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_CALIBRATOR_H
