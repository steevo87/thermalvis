/********************************************************************************
** Form generated from reading UI file 'mainwindow_streamer.ui'
**
** Created by: Qt User Interface Compiler version 5.3.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_STREAMER_H
#define UI_MAINWINDOW_STREAMER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow_streamer
{
public:
    QWidget *centralWidget;
    QCheckBox *debugMode;
    QCheckBox *verboseMode;
    QCheckBox *output8bit;
    QCheckBox *output16bit;
    QCheckBox *outputColor;
    QCheckBox *autoTemperature;
    QCheckBox *undistortImages;
    QComboBox *normMode;
    QComboBox *mapCode;
    QCheckBox *showExtremeColors;
    QLineEdit *desiredDegreesPerGraylevel;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow_streamer)
    {
        if (MainWindow_streamer->objectName().isEmpty())
            MainWindow_streamer->setObjectName(QStringLiteral("MainWindow_streamer"));
        MainWindow_streamer->resize(400, 300);
        centralWidget = new QWidget(MainWindow_streamer);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        debugMode = new QCheckBox(centralWidget);
        debugMode->setObjectName(QStringLiteral("debugMode"));
        debugMode->setGeometry(QRect(10, 10, 81, 17));
        verboseMode = new QCheckBox(centralWidget);
        verboseMode->setObjectName(QStringLiteral("verboseMode"));
        verboseMode->setGeometry(QRect(10, 30, 91, 17));
        output8bit = new QCheckBox(centralWidget);
        output8bit->setObjectName(QStringLiteral("output8bit"));
        output8bit->setGeometry(QRect(300, 10, 91, 17));
        output16bit = new QCheckBox(centralWidget);
        output16bit->setObjectName(QStringLiteral("output16bit"));
        output16bit->setGeometry(QRect(300, 30, 91, 17));
        outputColor = new QCheckBox(centralWidget);
        outputColor->setObjectName(QStringLiteral("outputColor"));
        outputColor->setGeometry(QRect(300, 50, 91, 17));
        autoTemperature = new QCheckBox(centralWidget);
        autoTemperature->setObjectName(QStringLiteral("autoTemperature"));
        autoTemperature->setGeometry(QRect(140, 10, 111, 17));
        undistortImages = new QCheckBox(centralWidget);
        undistortImages->setObjectName(QStringLiteral("undistortImages"));
        undistortImages->setGeometry(QRect(140, 30, 111, 17));
        normMode = new QComboBox(centralWidget);
        normMode->setObjectName(QStringLiteral("normMode"));
        normMode->setGeometry(QRect(10, 80, 151, 22));
        mapCode = new QComboBox(centralWidget);
        mapCode->setObjectName(QStringLiteral("mapCode"));
        mapCode->setGeometry(QRect(10, 110, 151, 22));
        showExtremeColors = new QCheckBox(centralWidget);
        showExtremeColors->setObjectName(QStringLiteral("showExtremeColors"));
        showExtremeColors->setGeometry(QRect(140, 50, 121, 17));
        desiredDegreesPerGraylevel = new QLineEdit(centralWidget);
        desiredDegreesPerGraylevel->setObjectName(QStringLiteral("desiredDegreesPerGraylevel"));
        desiredDegreesPerGraylevel->setGeometry(QRect(10, 140, 113, 20));
        MainWindow_streamer->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow_streamer);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 400, 21));
        MainWindow_streamer->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow_streamer);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow_streamer->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow_streamer);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow_streamer->setStatusBar(statusBar);

        retranslateUi(MainWindow_streamer);

        normMode->setCurrentIndex(3);
        mapCode->setCurrentIndex(7);


        QMetaObject::connectSlotsByName(MainWindow_streamer);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow_streamer)
    {
        MainWindow_streamer->setWindowTitle(QApplication::translate("MainWindow_streamer", "MainWindow_streamer", 0));
        debugMode->setText(QApplication::translate("MainWindow_streamer", "debugMode", 0));
        verboseMode->setText(QApplication::translate("MainWindow_streamer", "verboseMode", 0));
        output8bit->setText(QApplication::translate("MainWindow_streamer", "output8bit", 0));
        output16bit->setText(QApplication::translate("MainWindow_streamer", "output16bit", 0));
        outputColor->setText(QApplication::translate("MainWindow_streamer", "outputColor", 0));
        autoTemperature->setText(QApplication::translate("MainWindow_streamer", "autoTemperature", 0));
        undistortImages->setText(QApplication::translate("MainWindow_streamer", "undistortImages", 0));
        normMode->clear();
        normMode->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_streamer", "FULL_STRETCHING", 0)
         << QApplication::translate("MainWindow_streamer", "EQUALIZATION", 0)
         << QApplication::translate("MainWindow_streamer", "CENTRALIZED", 0)
         << QApplication::translate("MainWindow_streamer", "FIXED_TEMP_RANGE", 0)
         << QApplication::translate("MainWindow_streamer", "FIXED_TEMP_LIMITS", 0)
        );
        normMode->setCurrentText(QApplication::translate("MainWindow_streamer", "FIXED_TEMP_RANGE", 0));
        mapCode->clear();
        mapCode->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_streamer", "GRAYSCALE", 0)
         << QApplication::translate("MainWindow_streamer", "CIECOMP", 0)
         << QApplication::translate("MainWindow_streamer", "BLACKBODY", 0)
         << QApplication::translate("MainWindow_streamer", "RAINBOW", 0)
         << QApplication::translate("MainWindow_streamer", "IRON", 0)
         << QApplication::translate("MainWindow_streamer", "BLUERED", 0)
         << QApplication::translate("MainWindow_streamer", "JET", 0)
         << QApplication::translate("MainWindow_streamer", "CIELUV", 0)
         << QApplication::translate("MainWindow_streamer", "ICEIRON", 0)
         << QApplication::translate("MainWindow_streamer", "ICEFIRE", 0)
         << QApplication::translate("MainWindow_streamer", "REPEATED", 0)
         << QApplication::translate("MainWindow_streamer", "HIGHLIGHTED", 0)
        );
        mapCode->setCurrentText(QApplication::translate("MainWindow_streamer", "CIELUV", 0));
        showExtremeColors->setText(QApplication::translate("MainWindow_streamer", "showExtremeColors", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow_streamer: public Ui_MainWindow_streamer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_STREAMER_H
