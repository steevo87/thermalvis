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
    QLabel *mapCode_label;
    QLabel *normMode_label;
    QLabel *inputDatatype_label;
    QLabel *usbMode_label;
    QLabel *detectorMode_label;
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
        output8bit->setGeometry(QRect(10, 60, 91, 17));
        output16bit = new QCheckBox(centralWidget);
        output16bit->setObjectName(QStringLiteral("output16bit"));
        output16bit->setGeometry(QRect(10, 80, 91, 17));
        outputColor = new QCheckBox(centralWidget);
        outputColor->setObjectName(QStringLiteral("outputColor"));
        outputColor->setGeometry(QRect(10, 100, 91, 17));
        autoTemperature = new QCheckBox(centralWidget);
        autoTemperature->setObjectName(QStringLiteral("autoTemperature"));
        autoTemperature->setGeometry(QRect(10, 180, 111, 17));
        undistortImages = new QCheckBox(centralWidget);
        undistortImages->setObjectName(QStringLiteral("undistortImages"));
        undistortImages->setGeometry(QRect(10, 150, 111, 17));
        normMode = new QComboBox(centralWidget);
        normMode->setObjectName(QStringLiteral("normMode"));
        normMode->setGeometry(QRect(320, 130, 131, 22));
        mapCode = new QComboBox(centralWidget);
        mapCode->setObjectName(QStringLiteral("mapCode"));
        mapCode->setGeometry(QRect(180, 130, 91, 22));
        showExtremeColors = new QCheckBox(centralWidget);
        showExtremeColors->setObjectName(QStringLiteral("showExtremeColors"));
        showExtremeColors->setGeometry(QRect(10, 130, 121, 17));
        desiredDegreesPerGraylevel = new QLineEdit(centralWidget);
        desiredDegreesPerGraylevel->setObjectName(QStringLiteral("desiredDegreesPerGraylevel"));
        desiredDegreesPerGraylevel->setGeometry(QRect(290, 40, 61, 20));
        inputDatatype = new QComboBox(centralWidget);
        inputDatatype->setObjectName(QStringLiteral("inputDatatype"));
        inputDatatype->setGeometry(QRect(310, 180, 61, 22));
        detectorMode = new QComboBox(centralWidget);
        detectorMode->setObjectName(QStringLiteral("detectorMode"));
        detectorMode->setGeometry(QRect(310, 240, 61, 22));
        usbMode = new QComboBox(centralWidget);
        usbMode->setObjectName(QStringLiteral("usbMode"));
        usbMode->setGeometry(QRect(310, 210, 61, 22));
        maxReadAttempts = new QLineEdit(centralWidget);
        maxReadAttempts->setObjectName(QStringLiteral("maxReadAttempts"));
        maxReadAttempts->setGeometry(QRect(580, 40, 51, 20));
        maxNucInterval = new QLineEdit(centralWidget);
        maxNucInterval->setObjectName(QStringLiteral("maxNucInterval"));
        maxNucInterval->setGeometry(QRect(560, 180, 71, 20));
        zeroDegreesOffset = new QLineEdit(centralWidget);
        zeroDegreesOffset->setObjectName(QStringLiteral("zeroDegreesOffset"));
        zeroDegreesOffset->setGeometry(QRect(290, 70, 61, 20));
        degreesPerGraylevel = new QLineEdit(centralWidget);
        degreesPerGraylevel->setObjectName(QStringLiteral("degreesPerGraylevel"));
        degreesPerGraylevel->setGeometry(QRect(290, 10, 61, 20));
        maxReadAttempts_label = new QLabel(centralWidget);
        maxReadAttempts_label->setObjectName(QStringLiteral("maxReadAttempts_label"));
        maxReadAttempts_label->setGeometry(QRect(480, 40, 91, 16));
        maxReadAttempts_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        zeroDegreesOffset_label = new QLabel(centralWidget);
        zeroDegreesOffset_label->setObjectName(QStringLiteral("zeroDegreesOffset_label"));
        zeroDegreesOffset_label->setGeometry(QRect(180, 70, 101, 16));
        zeroDegreesOffset_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxNucInterval_label = new QLabel(centralWidget);
        maxNucInterval_label->setObjectName(QStringLiteral("maxNucInterval_label"));
        maxNucInterval_label->setGeometry(QRect(450, 180, 101, 16));
        maxNucInterval_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        degreesPerGraylevel_label = new QLabel(centralWidget);
        degreesPerGraylevel_label->setObjectName(QStringLiteral("degreesPerGraylevel_label"));
        degreesPerGraylevel_label->setGeometry(QRect(180, 10, 101, 20));
        degreesPerGraylevel_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        desiredDegreesPerGraylevel_label = new QLabel(centralWidget);
        desiredDegreesPerGraylevel_label->setObjectName(QStringLiteral("desiredDegreesPerGraylevel_label"));
        desiredDegreesPerGraylevel_label->setGeometry(QRect(120, 40, 161, 20));
        desiredDegreesPerGraylevel_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        framerate = new QLineEdit(centralWidget);
        framerate->setObjectName(QStringLiteral("framerate"));
        framerate->setGeometry(QRect(580, 10, 51, 20));
        framerate_label = new QLabel(centralWidget);
        framerate_label->setObjectName(QStringLiteral("framerate_label"));
        framerate_label->setGeometry(QRect(470, 10, 101, 20));
        framerate_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        threshFactor = new QLineEdit(centralWidget);
        threshFactor->setObjectName(QStringLiteral("threshFactor"));
        threshFactor->setGeometry(QRect(580, 140, 51, 20));
        threshFactor_label = new QLabel(centralWidget);
        threshFactor_label->setObjectName(QStringLiteral("threshFactor_label"));
        threshFactor_label->setGeometry(QRect(470, 140, 101, 20));
        threshFactor_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        normFactor = new QLineEdit(centralWidget);
        normFactor->setObjectName(QStringLiteral("normFactor"));
        normFactor->setGeometry(QRect(580, 80, 51, 20));
        normFactor_label = new QLabel(centralWidget);
        normFactor_label->setObjectName(QStringLiteral("normFactor_label"));
        normFactor_label->setGeometry(QRect(470, 80, 101, 20));
        normFactor_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        fusionFactor = new QLineEdit(centralWidget);
        fusionFactor->setObjectName(QStringLiteral("fusionFactor"));
        fusionFactor->setGeometry(QRect(580, 110, 51, 20));
        fusionFactor_label = new QLabel(centralWidget);
        fusionFactor_label->setObjectName(QStringLiteral("fusionFactor_label"));
        fusionFactor_label->setGeometry(QRect(470, 110, 101, 20));
        fusionFactor_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        serialPollingRate = new QLineEdit(centralWidget);
        serialPollingRate->setObjectName(QStringLiteral("serialPollingRate"));
        serialPollingRate->setGeometry(QRect(560, 240, 71, 20));
        serialPollingRate_label = new QLabel(centralWidget);
        serialPollingRate_label->setObjectName(QStringLiteral("serialPollingRate_label"));
        serialPollingRate_label->setGeometry(QRect(450, 240, 101, 20));
        serialPollingRate_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxNucThreshold = new QLineEdit(centralWidget);
        maxNucThreshold->setObjectName(QStringLiteral("maxNucThreshold"));
        maxNucThreshold->setGeometry(QRect(560, 210, 71, 20));
        maxNucThreshold_label = new QLabel(centralWidget);
        maxNucThreshold_label->setObjectName(QStringLiteral("maxNucThreshold_label"));
        maxNucThreshold_label->setGeometry(QRect(450, 210, 101, 20));
        maxNucThreshold_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        minTemp = new QLineEdit(centralWidget);
        minTemp->setObjectName(QStringLiteral("minTemp"));
        minTemp->setGeometry(QRect(60, 210, 61, 20));
        minTemp_label = new QLabel(centralWidget);
        minTemp_label->setObjectName(QStringLiteral("minTemp_label"));
        minTemp_label->setGeometry(QRect(0, 210, 51, 20));
        minTemp_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxTemp = new QLineEdit(centralWidget);
        maxTemp->setObjectName(QStringLiteral("maxTemp"));
        maxTemp->setGeometry(QRect(60, 240, 61, 20));
        maxTemp_label = new QLabel(centralWidget);
        maxTemp_label->setObjectName(QStringLiteral("maxTemp_label"));
        maxTemp_label->setGeometry(QRect(0, 240, 51, 20));
        maxTemp_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        mapCode_label = new QLabel(centralWidget);
        mapCode_label->setObjectName(QStringLiteral("mapCode_label"));
        mapCode_label->setGeometry(QRect(200, 110, 51, 20));
        mapCode_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        normMode_label = new QLabel(centralWidget);
        normMode_label->setObjectName(QStringLiteral("normMode_label"));
        normMode_label->setGeometry(QRect(360, 110, 51, 20));
        normMode_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        inputDatatype_label = new QLabel(centralWidget);
        inputDatatype_label->setObjectName(QStringLiteral("inputDatatype_label"));
        inputDatatype_label->setGeometry(QRect(220, 180, 81, 20));
        inputDatatype_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        usbMode_label = new QLabel(centralWidget);
        usbMode_label->setObjectName(QStringLiteral("usbMode_label"));
        usbMode_label->setGeometry(QRect(250, 210, 51, 20));
        usbMode_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        detectorMode_label = new QLabel(centralWidget);
        detectorMode_label->setObjectName(QStringLiteral("detectorMode_label"));
        detectorMode_label->setGeometry(QRect(220, 240, 81, 20));
        detectorMode_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
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
        desiredDegreesPerGraylevel->setText(QString());
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
        maxReadAttempts->setText(QString());
        maxNucInterval->setText(QString());
        zeroDegreesOffset->setText(QString());
        degreesPerGraylevel->setText(QString());
        maxReadAttempts_label->setText(QApplication::translate("MainWindow_streamer", "maxReadAttempts", 0));
        zeroDegreesOffset_label->setText(QApplication::translate("MainWindow_streamer", "zeroDegreesOffset", 0));
        maxNucInterval_label->setText(QApplication::translate("MainWindow_streamer", "maxNucInterval", 0));
        degreesPerGraylevel_label->setText(QApplication::translate("MainWindow_streamer", "degreesPerGraylevel", 0));
        desiredDegreesPerGraylevel_label->setText(QApplication::translate("MainWindow_streamer", "desiredDegreesPerGraylevel", 0));
        framerate->setText(QString());
        framerate_label->setText(QApplication::translate("MainWindow_streamer", "framerate", 0));
        threshFactor->setText(QString());
        threshFactor_label->setText(QApplication::translate("MainWindow_streamer", "threshFactor", 0));
        normFactor->setText(QString());
        normFactor_label->setText(QApplication::translate("MainWindow_streamer", "normFactor", 0));
        fusionFactor->setText(QString());
        fusionFactor_label->setText(QApplication::translate("MainWindow_streamer", "fusionFactor", 0));
        serialPollingRate->setText(QString());
        serialPollingRate_label->setText(QApplication::translate("MainWindow_streamer", "serialPollingRate", 0));
        maxNucThreshold->setText(QString());
        maxNucThreshold_label->setText(QApplication::translate("MainWindow_streamer", "maxNucThreshold", 0));
        minTemp->setText(QString());
        minTemp_label->setText(QApplication::translate("MainWindow_streamer", "minTemp", 0));
        maxTemp->setText(QString());
        maxTemp_label->setText(QApplication::translate("MainWindow_streamer", "maxTemp", 0));
        mapCode_label->setText(QApplication::translate("MainWindow_streamer", "mapCode", 0));
        normMode_label->setText(QApplication::translate("MainWindow_streamer", "normMode", 0));
        inputDatatype_label->setText(QApplication::translate("MainWindow_streamer", "inputDatatype", 0));
        usbMode_label->setText(QApplication::translate("MainWindow_streamer", "usbMode", 0));
        detectorMode_label->setText(QApplication::translate("MainWindow_streamer", "detectorMode", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow_streamer: public Ui_MainWindow_streamer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_STREAMER_H
