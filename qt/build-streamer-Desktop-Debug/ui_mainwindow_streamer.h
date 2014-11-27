/********************************************************************************
** Form generated from reading UI file 'mainwindow_streamer.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_STREAMER_H
#define UI_MAINWINDOW_STREAMER_H

#include <QVariant>
#include <QAction>
#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QWidget>

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
            MainWindow_streamer->setObjectName(QString::fromUtf8("MainWindow_streamer"));
        MainWindow_streamer->resize(640, 320);
        centralWidget = new QWidget(MainWindow_streamer);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        debugMode = new QCheckBox(centralWidget);
        debugMode->setObjectName(QString::fromUtf8("debugMode"));
        debugMode->setGeometry(QRect(10, 10, 121, 17));
        verboseMode = new QCheckBox(centralWidget);
        verboseMode->setObjectName(QString::fromUtf8("verboseMode"));
        verboseMode->setGeometry(QRect(10, 30, 131, 17));
        output8bit = new QCheckBox(centralWidget);
        output8bit->setObjectName(QString::fromUtf8("output8bit"));
        output8bit->setGeometry(QRect(10, 60, 131, 17));
        output16bit = new QCheckBox(centralWidget);
        output16bit->setObjectName(QString::fromUtf8("output16bit"));
        output16bit->setGeometry(QRect(10, 80, 121, 17));
        outputColor = new QCheckBox(centralWidget);
        outputColor->setObjectName(QString::fromUtf8("outputColor"));
        outputColor->setGeometry(QRect(10, 100, 131, 17));
        autoTemperature = new QCheckBox(centralWidget);
        autoTemperature->setObjectName(QString::fromUtf8("autoTemperature"));
        autoTemperature->setGeometry(QRect(10, 180, 161, 17));
        undistortImages = new QCheckBox(centralWidget);
        undistortImages->setObjectName(QString::fromUtf8("undistortImages"));
        undistortImages->setGeometry(QRect(10, 140, 151, 17));
        normMode = new QComboBox(centralWidget);
        normMode->setObjectName(QString::fromUtf8("normMode"));
        normMode->setGeometry(QRect(290, 130, 171, 22));
        mapCode = new QComboBox(centralWidget);
        mapCode->setObjectName(QString::fromUtf8("mapCode"));
        mapCode->setGeometry(QRect(170, 130, 111, 22));
        desiredDegreesPerGraylevel = new QLineEdit(centralWidget);
        desiredDegreesPerGraylevel->setObjectName(QString::fromUtf8("desiredDegreesPerGraylevel"));
        desiredDegreesPerGraylevel->setGeometry(QRect(360, 40, 61, 20));
        inputDatatype = new QComboBox(centralWidget);
        inputDatatype->setObjectName(QString::fromUtf8("inputDatatype"));
        inputDatatype->setGeometry(QRect(310, 180, 81, 22));
        detectorMode = new QComboBox(centralWidget);
        detectorMode->setObjectName(QString::fromUtf8("detectorMode"));
        detectorMode->setGeometry(QRect(310, 240, 81, 22));
        usbMode = new QComboBox(centralWidget);
        usbMode->setObjectName(QString::fromUtf8("usbMode"));
        usbMode->setGeometry(QRect(310, 210, 81, 22));
        maxReadAttempts = new QLineEdit(centralWidget);
        maxReadAttempts->setObjectName(QString::fromUtf8("maxReadAttempts"));
        maxReadAttempts->setGeometry(QRect(580, 40, 51, 20));
        maxNucInterval = new QLineEdit(centralWidget);
        maxNucInterval->setObjectName(QString::fromUtf8("maxNucInterval"));
        maxNucInterval->setGeometry(QRect(560, 180, 71, 20));
        zeroDegreesOffset = new QLineEdit(centralWidget);
        zeroDegreesOffset->setObjectName(QString::fromUtf8("zeroDegreesOffset"));
        zeroDegreesOffset->setGeometry(QRect(360, 70, 61, 20));
        degreesPerGraylevel = new QLineEdit(centralWidget);
        degreesPerGraylevel->setObjectName(QString::fromUtf8("degreesPerGraylevel"));
        degreesPerGraylevel->setGeometry(QRect(360, 10, 61, 20));
        maxReadAttempts_label = new QLabel(centralWidget);
        maxReadAttempts_label->setObjectName(QString::fromUtf8("maxReadAttempts_label"));
        maxReadAttempts_label->setGeometry(QRect(440, 40, 131, 20));
        maxReadAttempts_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        zeroDegreesOffset_label = new QLabel(centralWidget);
        zeroDegreesOffset_label->setObjectName(QString::fromUtf8("zeroDegreesOffset_label"));
        zeroDegreesOffset_label->setGeometry(QRect(220, 70, 131, 20));
        zeroDegreesOffset_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxNucInterval_label = new QLabel(centralWidget);
        maxNucInterval_label->setObjectName(QString::fromUtf8("maxNucInterval_label"));
        maxNucInterval_label->setGeometry(QRect(430, 180, 121, 20));
        maxNucInterval_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        degreesPerGraylevel_label = new QLabel(centralWidget);
        degreesPerGraylevel_label->setObjectName(QString::fromUtf8("degreesPerGraylevel_label"));
        degreesPerGraylevel_label->setGeometry(QRect(200, 10, 151, 20));
        degreesPerGraylevel_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        desiredDegreesPerGraylevel_label = new QLabel(centralWidget);
        desiredDegreesPerGraylevel_label->setObjectName(QString::fromUtf8("desiredDegreesPerGraylevel_label"));
        desiredDegreesPerGraylevel_label->setGeometry(QRect(150, 40, 201, 20));
        desiredDegreesPerGraylevel_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        framerate = new QLineEdit(centralWidget);
        framerate->setObjectName(QString::fromUtf8("framerate"));
        framerate->setGeometry(QRect(580, 10, 51, 20));
        framerate_label = new QLabel(centralWidget);
        framerate_label->setObjectName(QString::fromUtf8("framerate_label"));
        framerate_label->setGeometry(QRect(470, 10, 101, 20));
        framerate_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        threshFactor = new QLineEdit(centralWidget);
        threshFactor->setObjectName(QString::fromUtf8("threshFactor"));
        threshFactor->setGeometry(QRect(580, 140, 51, 20));
        threshFactor_label = new QLabel(centralWidget);
        threshFactor_label->setObjectName(QString::fromUtf8("threshFactor_label"));
        threshFactor_label->setGeometry(QRect(470, 140, 101, 20));
        threshFactor_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        normFactor = new QLineEdit(centralWidget);
        normFactor->setObjectName(QString::fromUtf8("normFactor"));
        normFactor->setGeometry(QRect(580, 80, 51, 20));
        normFactor_label = new QLabel(centralWidget);
        normFactor_label->setObjectName(QString::fromUtf8("normFactor_label"));
        normFactor_label->setGeometry(QRect(470, 80, 101, 20));
        normFactor_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        fusionFactor = new QLineEdit(centralWidget);
        fusionFactor->setObjectName(QString::fromUtf8("fusionFactor"));
        fusionFactor->setGeometry(QRect(580, 110, 51, 20));
        fusionFactor_label = new QLabel(centralWidget);
        fusionFactor_label->setObjectName(QString::fromUtf8("fusionFactor_label"));
        fusionFactor_label->setGeometry(QRect(470, 110, 101, 20));
        fusionFactor_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        serialPollingRate = new QLineEdit(centralWidget);
        serialPollingRate->setObjectName(QString::fromUtf8("serialPollingRate"));
        serialPollingRate->setGeometry(QRect(560, 240, 71, 20));
        serialPollingRate_label = new QLabel(centralWidget);
        serialPollingRate_label->setObjectName(QString::fromUtf8("serialPollingRate_label"));
        serialPollingRate_label->setGeometry(QRect(420, 240, 131, 20));
        serialPollingRate_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxNucThreshold = new QLineEdit(centralWidget);
        maxNucThreshold->setObjectName(QString::fromUtf8("maxNucThreshold"));
        maxNucThreshold->setGeometry(QRect(560, 210, 71, 20));
        maxNucThreshold_label = new QLabel(centralWidget);
        maxNucThreshold_label->setObjectName(QString::fromUtf8("maxNucThreshold_label"));
        maxNucThreshold_label->setGeometry(QRect(420, 210, 131, 20));
        maxNucThreshold_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        minTemp = new QLineEdit(centralWidget);
        minTemp->setObjectName(QString::fromUtf8("minTemp"));
        minTemp->setGeometry(QRect(80, 210, 61, 20));
        minTemp_label = new QLabel(centralWidget);
        minTemp_label->setObjectName(QString::fromUtf8("minTemp_label"));
        minTemp_label->setGeometry(QRect(0, 210, 71, 20));
        minTemp_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxTemp = new QLineEdit(centralWidget);
        maxTemp->setObjectName(QString::fromUtf8("maxTemp"));
        maxTemp->setGeometry(QRect(80, 240, 61, 20));
        maxTemp_label = new QLabel(centralWidget);
        maxTemp_label->setObjectName(QString::fromUtf8("maxTemp_label"));
        maxTemp_label->setGeometry(QRect(0, 240, 71, 20));
        maxTemp_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        mapCode_label = new QLabel(centralWidget);
        mapCode_label->setObjectName(QString::fromUtf8("mapCode_label"));
        mapCode_label->setGeometry(QRect(190, 110, 71, 20));
        mapCode_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        normMode_label = new QLabel(centralWidget);
        normMode_label->setObjectName(QString::fromUtf8("normMode_label"));
        normMode_label->setGeometry(QRect(330, 110, 81, 20));
        normMode_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        inputDatatype_label = new QLabel(centralWidget);
        inputDatatype_label->setObjectName(QString::fromUtf8("inputDatatype_label"));
        inputDatatype_label->setGeometry(QRect(200, 180, 101, 20));
        inputDatatype_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        usbMode_label = new QLabel(centralWidget);
        usbMode_label->setObjectName(QString::fromUtf8("usbMode_label"));
        usbMode_label->setGeometry(QRect(230, 210, 71, 20));
        usbMode_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        detectorMode_label = new QLabel(centralWidget);
        detectorMode_label->setObjectName(QString::fromUtf8("detectorMode_label"));
        detectorMode_label->setGeometry(QRect(200, 240, 101, 20));
        detectorMode_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        MainWindow_streamer->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow_streamer);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 640, 25));
        MainWindow_streamer->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow_streamer);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow_streamer->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow_streamer);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
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
        normMode->setCurrentIndex(normMode->findText("FIXED_TEMP_RANGE"));
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
        mapCode->setCurrentIndex(mapCode->findText("CIELUV"));
        desiredDegreesPerGraylevel->setText(QString());
        inputDatatype->clear();
        inputDatatype->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_streamer", "8BIT", 0)
         << QApplication::translate("MainWindow_streamer", "RAW", 0)
         << QApplication::translate("MainWindow_streamer", "MULTIMODAL", 0)
         << QApplication::translate("MainWindow_streamer", "DEPTH", 0)
        );
        inputDatatype->setCurrentIndex(inputDatatype->findText("RAW"));
        detectorMode->clear();
        detectorMode->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_streamer", "RAW", 0)
         << QApplication::translate("MainWindow_streamer", "LUM", 0)
         << QApplication::translate("MainWindow_streamer", "INS", 0)
         << QApplication::translate("MainWindow_streamer", "RAD", 0)
         << QApplication::translate("MainWindow_streamer", "TMP", 0)
        );
        detectorMode->setCurrentIndex(detectorMode->findText("INS"));
        usbMode->clear();
        usbMode->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_streamer", "USB_16", 0)
         << QApplication::translate("MainWindow_streamer", "USB_8", 0)
        );
        usbMode->setCurrentIndex(usbMode->findText("USB_16"));
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
