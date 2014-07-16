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
#include <QtWidgets/QLabel>
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
    QComboBox *inputDatatype;
    QComboBox *detectorMode;
    QComboBox *usbMode;
    QLineEdit *maxReadAttempts;
    QLineEdit *maxNucInterval;
    QLineEdit *zeroDegreesOffset;
    QLineEdit *degreesPerGraylevel;
    QLabel *maxReadAttempts_label;
    QLabel *zeroDegreesOffset_label;
    QLabel *maxNucInterval_label;
    QLabel *degreesPerGraylevel_label;
    QLabel *desiredDegreesPerGraylevel_label;
    QLineEdit *framerate;
    QLabel *framerate_label;
    QLineEdit *threshFactor;
    QLabel *threshFactor_label;
    QLineEdit *normFactor;
    QLabel *normFactor_label;
    QLineEdit *fusionFactor;
    QLabel *fusionFactor_label;
    QLineEdit *serialPollingRate;
    QLabel *serialPollingRate_label;
    QLineEdit *maxNucThreshold;
    QLabel *maxNucThreshold_label;
    QLineEdit *minTemp;
    QLabel *minTemp_label;
    QLineEdit *maxTemp;
    QLabel *maxTemp_label;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow_streamer)
    {
        if (MainWindow_streamer->objectName().isEmpty())
            MainWindow_streamer->setObjectName(QStringLiteral("MainWindow_streamer"));
        MainWindow_streamer->resize(640, 320);
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
        desiredDegreesPerGraylevel->setGeometry(QRect(510, 230, 113, 20));
        inputDatatype = new QComboBox(centralWidget);
        inputDatatype->setObjectName(QStringLiteral("inputDatatype"));
        inputDatatype->setGeometry(QRect(10, 140, 151, 22));
        detectorMode = new QComboBox(centralWidget);
        detectorMode->setObjectName(QStringLiteral("detectorMode"));
        detectorMode->setGeometry(QRect(10, 170, 151, 22));
        usbMode = new QComboBox(centralWidget);
        usbMode->setObjectName(QStringLiteral("usbMode"));
        usbMode->setGeometry(QRect(10, 200, 151, 22));
        maxReadAttempts = new QLineEdit(centralWidget);
        maxReadAttempts->setObjectName(QStringLiteral("maxReadAttempts"));
        maxReadAttempts->setGeometry(QRect(280, 100, 113, 20));
        maxNucInterval = new QLineEdit(centralWidget);
        maxNucInterval->setObjectName(QStringLiteral("maxNucInterval"));
        maxNucInterval->setGeometry(QRect(280, 160, 113, 20));
        zeroDegreesOffset = new QLineEdit(centralWidget);
        zeroDegreesOffset->setObjectName(QStringLiteral("zeroDegreesOffset"));
        zeroDegreesOffset->setGeometry(QRect(280, 130, 113, 20));
        degreesPerGraylevel = new QLineEdit(centralWidget);
        degreesPerGraylevel->setObjectName(QStringLiteral("degreesPerGraylevel"));
        degreesPerGraylevel->setGeometry(QRect(510, 200, 113, 20));
        maxReadAttempts_label = new QLabel(centralWidget);
        maxReadAttempts_label->setObjectName(QStringLiteral("maxReadAttempts_label"));
        maxReadAttempts_label->setGeometry(QRect(180, 100, 91, 16));
        maxReadAttempts_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        zeroDegreesOffset_label = new QLabel(centralWidget);
        zeroDegreesOffset_label->setObjectName(QStringLiteral("zeroDegreesOffset_label"));
        zeroDegreesOffset_label->setGeometry(QRect(170, 130, 101, 16));
        zeroDegreesOffset_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxNucInterval_label = new QLabel(centralWidget);
        maxNucInterval_label->setObjectName(QStringLiteral("maxNucInterval_label"));
        maxNucInterval_label->setGeometry(QRect(170, 160, 101, 16));
        maxNucInterval_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        degreesPerGraylevel_label = new QLabel(centralWidget);
        degreesPerGraylevel_label->setObjectName(QStringLiteral("degreesPerGraylevel_label"));
        degreesPerGraylevel_label->setGeometry(QRect(400, 200, 101, 20));
        degreesPerGraylevel_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        desiredDegreesPerGraylevel_label = new QLabel(centralWidget);
        desiredDegreesPerGraylevel_label->setObjectName(QStringLiteral("desiredDegreesPerGraylevel_label"));
        desiredDegreesPerGraylevel_label->setGeometry(QRect(340, 230, 161, 20));
        desiredDegreesPerGraylevel_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        framerate = new QLineEdit(centralWidget);
        framerate->setObjectName(QStringLiteral("framerate"));
        framerate->setGeometry(QRect(510, 10, 113, 20));
        framerate_label = new QLabel(centralWidget);
        framerate_label->setObjectName(QStringLiteral("framerate_label"));
        framerate_label->setGeometry(QRect(400, 10, 101, 20));
        framerate_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        threshFactor = new QLineEdit(centralWidget);
        threshFactor->setObjectName(QStringLiteral("threshFactor"));
        threshFactor->setGeometry(QRect(510, 40, 113, 20));
        threshFactor_label = new QLabel(centralWidget);
        threshFactor_label->setObjectName(QStringLiteral("threshFactor_label"));
        threshFactor_label->setGeometry(QRect(400, 40, 101, 20));
        threshFactor_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        normFactor = new QLineEdit(centralWidget);
        normFactor->setObjectName(QStringLiteral("normFactor"));
        normFactor->setGeometry(QRect(510, 70, 113, 20));
        normFactor_label = new QLabel(centralWidget);
        normFactor_label->setObjectName(QStringLiteral("normFactor_label"));
        normFactor_label->setGeometry(QRect(400, 70, 101, 20));
        normFactor_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        fusionFactor = new QLineEdit(centralWidget);
        fusionFactor->setObjectName(QStringLiteral("fusionFactor"));
        fusionFactor->setGeometry(QRect(510, 100, 113, 20));
        fusionFactor_label = new QLabel(centralWidget);
        fusionFactor_label->setObjectName(QStringLiteral("fusionFactor_label"));
        fusionFactor_label->setGeometry(QRect(400, 100, 101, 20));
        fusionFactor_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        serialPollingRate = new QLineEdit(centralWidget);
        serialPollingRate->setObjectName(QStringLiteral("serialPollingRate"));
        serialPollingRate->setGeometry(QRect(510, 130, 113, 20));
        serialPollingRate_label = new QLabel(centralWidget);
        serialPollingRate_label->setObjectName(QStringLiteral("serialPollingRate_label"));
        serialPollingRate_label->setGeometry(QRect(400, 130, 101, 20));
        serialPollingRate_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxNucThreshold = new QLineEdit(centralWidget);
        maxNucThreshold->setObjectName(QStringLiteral("maxNucThreshold"));
        maxNucThreshold->setGeometry(QRect(510, 160, 113, 20));
        maxNucThreshold_label = new QLabel(centralWidget);
        maxNucThreshold_label->setObjectName(QStringLiteral("maxNucThreshold_label"));
        maxNucThreshold_label->setGeometry(QRect(400, 160, 101, 20));
        maxNucThreshold_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        minTemp = new QLineEdit(centralWidget);
        minTemp->setObjectName(QStringLiteral("minTemp"));
        minTemp->setGeometry(QRect(280, 200, 61, 20));
        minTemp_label = new QLabel(centralWidget);
        minTemp_label->setObjectName(QStringLiteral("minTemp_label"));
        minTemp_label->setGeometry(QRect(220, 200, 51, 20));
        minTemp_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxTemp = new QLineEdit(centralWidget);
        maxTemp->setObjectName(QStringLiteral("maxTemp"));
        maxTemp->setGeometry(QRect(280, 230, 61, 20));
        maxTemp_label = new QLabel(centralWidget);
        maxTemp_label->setObjectName(QStringLiteral("maxTemp_label"));
        maxTemp_label->setGeometry(QRect(220, 230, 51, 20));
        maxTemp_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        MainWindow_streamer->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow_streamer);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 640, 21));
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
        inputDatatype->setCurrentIndex(1);
        detectorMode->setCurrentIndex(2);
        usbMode->setCurrentIndex(0);


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
        desiredDegreesPerGraylevel->setText(QApplication::translate("MainWindow_streamer", "0.05", 0));
        inputDatatype->clear();
        inputDatatype->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_streamer", "8BIT", 0)
         << QApplication::translate("MainWindow_streamer", "RAW", 0)
         << QApplication::translate("MainWindow_streamer", "MULTIMODAL", 0)
         << QApplication::translate("MainWindow_streamer", "DEPTH", 0)
        );
        inputDatatype->setCurrentText(QApplication::translate("MainWindow_streamer", "RAW", 0));
        detectorMode->clear();
        detectorMode->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_streamer", "RAW", 0)
         << QApplication::translate("MainWindow_streamer", "LUM", 0)
         << QApplication::translate("MainWindow_streamer", "INS", 0)
         << QApplication::translate("MainWindow_streamer", "RAD", 0)
         << QApplication::translate("MainWindow_streamer", "TMP", 0)
        );
        detectorMode->setCurrentText(QApplication::translate("MainWindow_streamer", "INS", 0));
        usbMode->clear();
        usbMode->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_streamer", "USB_16", 0)
         << QApplication::translate("MainWindow_streamer", "USB_8", 0)
        );
        usbMode->setCurrentText(QApplication::translate("MainWindow_streamer", "USB_16", 0));
        maxReadAttempts->setText(QApplication::translate("MainWindow_streamer", "0", 0));
        maxNucInterval->setText(QApplication::translate("MainWindow_streamer", "45", 0));
        zeroDegreesOffset->setText(QApplication::translate("MainWindow_streamer", "0", 0));
        degreesPerGraylevel->setText(QApplication::translate("MainWindow_streamer", "0.01", 0));
        maxReadAttempts_label->setText(QApplication::translate("MainWindow_streamer", "maxReadAttempts", 0));
        zeroDegreesOffset_label->setText(QApplication::translate("MainWindow_streamer", "zeroDegreesOffset", 0));
        maxNucInterval_label->setText(QApplication::translate("MainWindow_streamer", "maxNucInterval", 0));
        degreesPerGraylevel_label->setText(QApplication::translate("MainWindow_streamer", "degreesPerGraylevel", 0));
        desiredDegreesPerGraylevel_label->setText(QApplication::translate("MainWindow_streamer", "desiredDegreesPerGraylevel", 0));
        framerate->setText(QApplication::translate("MainWindow_streamer", "-1.0", 0));
        framerate_label->setText(QApplication::translate("MainWindow_streamer", "framerate", 0));
        threshFactor->setText(QApplication::translate("MainWindow_streamer", "0.0", 0));
        threshFactor_label->setText(QApplication::translate("MainWindow_streamer", "threshFactor", 0));
        normFactor->setText(QApplication::translate("MainWindow_streamer", "0.2", 0));
        normFactor_label->setText(QApplication::translate("MainWindow_streamer", "normFactor", 0));
        fusionFactor->setText(QApplication::translate("MainWindow_streamer", "0.6", 0));
        fusionFactor_label->setText(QApplication::translate("MainWindow_streamer", "fusionFactor", 0));
        serialPollingRate->setText(QApplication::translate("MainWindow_streamer", "25.0", 0));
        serialPollingRate_label->setText(QApplication::translate("MainWindow_streamer", "serialPollingRate", 0));
        maxNucThreshold->setText(QApplication::translate("MainWindow_streamer", "0.2", 0));
        maxNucThreshold_label->setText(QApplication::translate("MainWindow_streamer", "maxNucThreshold", 0));
        minTemp->setText(QApplication::translate("MainWindow_streamer", "25.0", 0));
        minTemp_label->setText(QApplication::translate("MainWindow_streamer", "minTemp", 0));
        maxTemp->setText(QApplication::translate("MainWindow_streamer", "35.0", 0));
        maxTemp_label->setText(QApplication::translate("MainWindow_streamer", "maxTemp", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow_streamer: public Ui_MainWindow_streamer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_STREAMER_H
