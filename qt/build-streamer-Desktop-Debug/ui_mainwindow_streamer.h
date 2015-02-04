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
#include <QGroupBox>
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
    QGroupBox *groupBox_Output;
    QCheckBox *outputColor;
    QCheckBox *output16bit;
    QCheckBox *output8bit;
    QGroupBox *groupBox_Scaling;
    QLabel *minTemp_label;
    QLabel *maxTemp_label;
    QLineEdit *maxTemp;
    QCheckBox *autoTemperature;
    QLineEdit *minTemp;
    QGroupBox *groupBox_Graylevel;
    QLineEdit *zeroDegreesOffset;
    QLabel *degreesPerGraylevel_label;
    QLabel *desiredDegreesPerGraylevel_label;
    QLineEdit *desiredDegreesPerGraylevel;
    QLineEdit *degreesPerGraylevel;
    QLabel *zeroDegreesOffset_label;
    QGroupBox *groupBox_ImageProcessing;
    QLineEdit *threshFactor;
    QLabel *threshFactor_label;
    QLineEdit *normFactor;
    QLabel *fusionFactor_label;
    QLabel *normFactor_label;
    QLineEdit *fusionFactor;
    QCheckBox *undistortImages;
    QComboBox *denoisingMethod;
    QLabel *denoising_label;
    QLabel *normMode_label;
    QComboBox *normMode;
    QComboBox *mapCode;
    QLabel *mapCode_label;
    QGroupBox *groupBox_Frames;
    QLineEdit *maxReadAttempts;
    QLabel *framerate_label;
    QLabel *maxReadAttempts_label;
    QLineEdit *framerate;
    QGroupBox *groupBox_Comms;
    QLineEdit *serialPollingRate;
    QLabel *serialPollingRate_label;
    QLineEdit *maxNucInterval;
    QLabel *maxNucThreshold_label;
    QLineEdit *maxNucThreshold;
    QLabel *maxNucInterval_label;
    QGroupBox *groupBox_Input;
    QComboBox *detectorMode;
    QLabel *detectorMode_label;
    QComboBox *inputDatatype;
    QComboBox *usbMode;
    QLabel *inputDatatype_label;
    QLabel *usbMode_label;
    QGroupBox *groupBox_Development;
    QCheckBox *debugMode;
    QCheckBox *verboseMode;
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
        groupBox_Output = new QGroupBox(centralWidget);
        groupBox_Output->setObjectName(QString::fromUtf8("groupBox_Output"));
        groupBox_Output->setGeometry(QRect(200, 4, 71, 91));
        outputColor = new QCheckBox(groupBox_Output);
        outputColor->setObjectName(QString::fromUtf8("outputColor"));
        outputColor->setGeometry(QRect(10, 68, 131, 17));
        output16bit = new QCheckBox(groupBox_Output);
        output16bit->setObjectName(QString::fromUtf8("output16bit"));
        output16bit->setGeometry(QRect(10, 44, 121, 17));
        output8bit = new QCheckBox(groupBox_Output);
        output8bit->setObjectName(QString::fromUtf8("output8bit"));
        output8bit->setGeometry(QRect(10, 20, 131, 17));
        outputColor->raise();
        output16bit->raise();
        output8bit->raise();
        groupBox_Scaling = new QGroupBox(centralWidget);
        groupBox_Scaling->setObjectName(QString::fromUtf8("groupBox_Scaling"));
        groupBox_Scaling->setGeometry(QRect(200, 194, 231, 71));
        minTemp_label = new QLabel(groupBox_Scaling);
        minTemp_label->setObjectName(QString::fromUtf8("minTemp_label"));
        minTemp_label->setGeometry(QRect(-19, 41, 71, 20));
        minTemp_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxTemp_label = new QLabel(groupBox_Scaling);
        maxTemp_label->setObjectName(QString::fromUtf8("maxTemp_label"));
        maxTemp_label->setGeometry(QRect(91, 41, 71, 20));
        maxTemp_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxTemp = new QLineEdit(groupBox_Scaling);
        maxTemp->setObjectName(QString::fromUtf8("maxTemp"));
        maxTemp->setGeometry(QRect(171, 41, 41, 20));
        autoTemperature = new QCheckBox(groupBox_Scaling);
        autoTemperature->setObjectName(QString::fromUtf8("autoTemperature"));
        autoTemperature->setGeometry(QRect(10, 21, 161, 17));
        minTemp = new QLineEdit(groupBox_Scaling);
        minTemp->setObjectName(QString::fromUtf8("minTemp"));
        minTemp->setGeometry(QRect(61, 41, 41, 20));
        groupBox_Graylevel = new QGroupBox(centralWidget);
        groupBox_Graylevel->setObjectName(QString::fromUtf8("groupBox_Graylevel"));
        groupBox_Graylevel->setGeometry(QRect(200, 100, 231, 91));
        zeroDegreesOffset = new QLineEdit(groupBox_Graylevel);
        zeroDegreesOffset->setObjectName(QString::fromUtf8("zeroDegreesOffset"));
        zeroDegreesOffset->setGeometry(QRect(150, 64, 61, 20));
        degreesPerGraylevel_label = new QLabel(groupBox_Graylevel);
        degreesPerGraylevel_label->setObjectName(QString::fromUtf8("degreesPerGraylevel_label"));
        degreesPerGraylevel_label->setGeometry(QRect(20, 14, 121, 20));
        degreesPerGraylevel_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        desiredDegreesPerGraylevel_label = new QLabel(groupBox_Graylevel);
        desiredDegreesPerGraylevel_label->setObjectName(QString::fromUtf8("desiredDegreesPerGraylevel_label"));
        desiredDegreesPerGraylevel_label->setGeometry(QRect(0, 39, 141, 20));
        desiredDegreesPerGraylevel_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        desiredDegreesPerGraylevel = new QLineEdit(groupBox_Graylevel);
        desiredDegreesPerGraylevel->setObjectName(QString::fromUtf8("desiredDegreesPerGraylevel"));
        desiredDegreesPerGraylevel->setGeometry(QRect(150, 39, 61, 20));
        degreesPerGraylevel = new QLineEdit(groupBox_Graylevel);
        degreesPerGraylevel->setObjectName(QString::fromUtf8("degreesPerGraylevel"));
        degreesPerGraylevel->setGeometry(QRect(150, 14, 61, 20));
        zeroDegreesOffset_label = new QLabel(groupBox_Graylevel);
        zeroDegreesOffset_label->setObjectName(QString::fromUtf8("zeroDegreesOffset_label"));
        zeroDegreesOffset_label->setGeometry(QRect(10, 64, 131, 20));
        zeroDegreesOffset_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        groupBox_ImageProcessing = new QGroupBox(centralWidget);
        groupBox_ImageProcessing->setObjectName(QString::fromUtf8("groupBox_ImageProcessing"));
        groupBox_ImageProcessing->setGeometry(QRect(440, 4, 191, 261));
        threshFactor = new QLineEdit(groupBox_ImageProcessing);
        threshFactor->setObjectName(QString::fromUtf8("threshFactor"));
        threshFactor->setGeometry(QRect(100, 70, 51, 20));
        threshFactor_label = new QLabel(groupBox_ImageProcessing);
        threshFactor_label->setObjectName(QString::fromUtf8("threshFactor_label"));
        threshFactor_label->setGeometry(QRect(-10, 70, 101, 20));
        threshFactor_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        normFactor = new QLineEdit(groupBox_ImageProcessing);
        normFactor->setObjectName(QString::fromUtf8("normFactor"));
        normFactor->setGeometry(QRect(100, 19, 51, 20));
        fusionFactor_label = new QLabel(groupBox_ImageProcessing);
        fusionFactor_label->setObjectName(QString::fromUtf8("fusionFactor_label"));
        fusionFactor_label->setGeometry(QRect(-10, 44, 101, 20));
        fusionFactor_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        normFactor_label = new QLabel(groupBox_ImageProcessing);
        normFactor_label->setObjectName(QString::fromUtf8("normFactor_label"));
        normFactor_label->setGeometry(QRect(-10, 19, 101, 20));
        normFactor_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        fusionFactor = new QLineEdit(groupBox_ImageProcessing);
        fusionFactor->setObjectName(QString::fromUtf8("fusionFactor"));
        fusionFactor->setGeometry(QRect(100, 44, 51, 20));
        undistortImages = new QCheckBox(groupBox_ImageProcessing);
        undistortImages->setObjectName(QString::fromUtf8("undistortImages"));
        undistortImages->setGeometry(QRect(50, 100, 151, 17));
        denoisingMethod = new QComboBox(groupBox_ImageProcessing);
        denoisingMethod->setObjectName(QString::fromUtf8("denoisingMethod"));
        denoisingMethod->setGeometry(QRect(70, 140, 111, 22));
        denoising_label = new QLabel(groupBox_ImageProcessing);
        denoising_label->setObjectName(QString::fromUtf8("denoising_label"));
        denoising_label->setGeometry(QRect(10, 140, 51, 20));
        denoising_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        normMode_label = new QLabel(groupBox_ImageProcessing);
        normMode_label->setObjectName(QString::fromUtf8("normMode_label"));
        normMode_label->setGeometry(QRect(40, 210, 81, 20));
        normMode_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        normMode = new QComboBox(groupBox_ImageProcessing);
        normMode->setObjectName(QString::fromUtf8("normMode"));
        normMode->setGeometry(QRect(10, 230, 171, 22));
        mapCode = new QComboBox(groupBox_ImageProcessing);
        mapCode->setObjectName(QString::fromUtf8("mapCode"));
        mapCode->setGeometry(QRect(70, 170, 111, 22));
        mapCode_label = new QLabel(groupBox_ImageProcessing);
        mapCode_label->setObjectName(QString::fromUtf8("mapCode_label"));
        mapCode_label->setGeometry(QRect(-10, 170, 71, 20));
        mapCode_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        groupBox_Frames = new QGroupBox(centralWidget);
        groupBox_Frames->setObjectName(QString::fromUtf8("groupBox_Frames"));
        groupBox_Frames->setGeometry(QRect(280, 4, 151, 91));
        maxReadAttempts = new QLineEdit(groupBox_Frames);
        maxReadAttempts->setObjectName(QString::fromUtf8("maxReadAttempts"));
        maxReadAttempts->setGeometry(QRect(110, 50, 31, 20));
        framerate_label = new QLabel(groupBox_Frames);
        framerate_label->setObjectName(QString::fromUtf8("framerate_label"));
        framerate_label->setGeometry(QRect(0, 20, 101, 20));
        framerate_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxReadAttempts_label = new QLabel(groupBox_Frames);
        maxReadAttempts_label->setObjectName(QString::fromUtf8("maxReadAttempts_label"));
        maxReadAttempts_label->setGeometry(QRect(-30, 50, 131, 20));
        maxReadAttempts_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        framerate = new QLineEdit(groupBox_Frames);
        framerate->setObjectName(QString::fromUtf8("framerate"));
        framerate->setGeometry(QRect(110, 20, 31, 20));
        groupBox_Comms = new QGroupBox(centralWidget);
        groupBox_Comms->setObjectName(QString::fromUtf8("groupBox_Comms"));
        groupBox_Comms->setGeometry(QRect(10, 100, 181, 91));
        serialPollingRate = new QLineEdit(groupBox_Comms);
        serialPollingRate->setObjectName(QString::fromUtf8("serialPollingRate"));
        serialPollingRate->setGeometry(QRect(109, 62, 51, 20));
        serialPollingRate_label = new QLabel(groupBox_Comms);
        serialPollingRate_label->setObjectName(QString::fromUtf8("serialPollingRate_label"));
        serialPollingRate_label->setGeometry(QRect(10, 62, 91, 20));
        serialPollingRate_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxNucInterval = new QLineEdit(groupBox_Comms);
        maxNucInterval->setObjectName(QString::fromUtf8("maxNucInterval"));
        maxNucInterval->setGeometry(QRect(109, 13, 51, 20));
        maxNucThreshold_label = new QLabel(groupBox_Comms);
        maxNucThreshold_label->setObjectName(QString::fromUtf8("maxNucThreshold_label"));
        maxNucThreshold_label->setGeometry(QRect(10, 37, 91, 20));
        maxNucThreshold_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxNucThreshold = new QLineEdit(groupBox_Comms);
        maxNucThreshold->setObjectName(QString::fromUtf8("maxNucThreshold"));
        maxNucThreshold->setGeometry(QRect(109, 37, 51, 20));
        maxNucInterval_label = new QLabel(groupBox_Comms);
        maxNucInterval_label->setObjectName(QString::fromUtf8("maxNucInterval_label"));
        maxNucInterval_label->setGeometry(QRect(10, 13, 91, 20));
        maxNucInterval_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        groupBox_Input = new QGroupBox(centralWidget);
        groupBox_Input->setObjectName(QString::fromUtf8("groupBox_Input"));
        groupBox_Input->setGeometry(QRect(10, 4, 181, 91));
        detectorMode = new QComboBox(groupBox_Input);
        detectorMode->setObjectName(QString::fromUtf8("detectorMode"));
        detectorMode->setGeometry(QRect(89, 62, 81, 22));
        detectorMode_label = new QLabel(groupBox_Input);
        detectorMode_label->setObjectName(QString::fromUtf8("detectorMode_label"));
        detectorMode_label->setGeometry(QRect(-21, 62, 101, 20));
        detectorMode_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        inputDatatype = new QComboBox(groupBox_Input);
        inputDatatype->setObjectName(QString::fromUtf8("inputDatatype"));
        inputDatatype->setGeometry(QRect(89, 12, 81, 22));
        usbMode = new QComboBox(groupBox_Input);
        usbMode->setObjectName(QString::fromUtf8("usbMode"));
        usbMode->setGeometry(QRect(89, 37, 81, 22));
        inputDatatype_label = new QLabel(groupBox_Input);
        inputDatatype_label->setObjectName(QString::fromUtf8("inputDatatype_label"));
        inputDatatype_label->setGeometry(QRect(-21, 12, 101, 20));
        inputDatatype_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        usbMode_label = new QLabel(groupBox_Input);
        usbMode_label->setObjectName(QString::fromUtf8("usbMode_label"));
        usbMode_label->setGeometry(QRect(9, 37, 71, 20));
        usbMode_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        groupBox_Development = new QGroupBox(centralWidget);
        groupBox_Development->setObjectName(QString::fromUtf8("groupBox_Development"));
        groupBox_Development->setGeometry(QRect(50, 200, 101, 61));
        debugMode = new QCheckBox(groupBox_Development);
        debugMode->setObjectName(QString::fromUtf8("debugMode"));
        debugMode->setGeometry(QRect(10, 20, 91, 17));
        verboseMode = new QCheckBox(groupBox_Development);
        verboseMode->setObjectName(QString::fromUtf8("verboseMode"));
        verboseMode->setGeometry(QRect(10, 40, 91, 17));
        MainWindow_streamer->setCentralWidget(centralWidget);
        groupBox_Output->raise();
        groupBox_Scaling->raise();
        groupBox_Graylevel->raise();
        groupBox_ImageProcessing->raise();
        groupBox_Frames->raise();
        groupBox_Comms->raise();
        groupBox_Input->raise();
        groupBox_Development->raise();
        debugMode->raise();
        menuBar = new QMenuBar(MainWindow_streamer);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 640, 21));
        MainWindow_streamer->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow_streamer);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow_streamer->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow_streamer);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow_streamer->setStatusBar(statusBar);

        retranslateUi(MainWindow_streamer);

        denoisingMethod->setCurrentIndex(0);
        normMode->setCurrentIndex(3);
        mapCode->setCurrentIndex(7);
        detectorMode->setCurrentIndex(2);
        inputDatatype->setCurrentIndex(1);
        usbMode->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow_streamer);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow_streamer)
    {
        MainWindow_streamer->setWindowTitle(QApplication::translate("MainWindow_streamer", "MainWindow_streamer", 0));
        groupBox_Output->setTitle(QApplication::translate("MainWindow_streamer", "Output", 0));
        outputColor->setText(QApplication::translate("MainWindow_streamer", "Color", 0));
        output16bit->setText(QApplication::translate("MainWindow_streamer", "16-bit", 0));
        output8bit->setText(QApplication::translate("MainWindow_streamer", "8-bit", 0));
        groupBox_Scaling->setTitle(QApplication::translate("MainWindow_streamer", "Temperature Scaling", 0));
        minTemp_label->setText(QApplication::translate("MainWindow_streamer", "minTemp", 0));
        maxTemp_label->setText(QApplication::translate("MainWindow_streamer", "maxTemp", 0));
        maxTemp->setText(QString());
        autoTemperature->setText(QApplication::translate("MainWindow_streamer", "autoTemperature", 0));
        minTemp->setText(QString());
        groupBox_Graylevel->setTitle(QApplication::translate("MainWindow_streamer", "Graylevel Scaling", 0));
        zeroDegreesOffset->setText(QString());
        degreesPerGraylevel_label->setText(QApplication::translate("MainWindow_streamer", "degreesPerGraylevel", 0));
        desiredDegreesPerGraylevel_label->setText(QApplication::translate("MainWindow_streamer", "desiredDegreesPerGraylevel", 0));
        desiredDegreesPerGraylevel->setText(QString());
        degreesPerGraylevel->setText(QString());
        zeroDegreesOffset_label->setText(QApplication::translate("MainWindow_streamer", "zeroDegreesOffset", 0));
        groupBox_ImageProcessing->setTitle(QApplication::translate("MainWindow_streamer", "Image Processing", 0));
        threshFactor->setText(QString());
        threshFactor_label->setText(QApplication::translate("MainWindow_streamer", "threshFactor", 0));
        normFactor->setText(QString());
        fusionFactor_label->setText(QApplication::translate("MainWindow_streamer", "fusionFactor", 0));
        normFactor_label->setText(QApplication::translate("MainWindow_streamer", "normFactor", 0));
        fusionFactor->setText(QString());
        undistortImages->setText(QApplication::translate("MainWindow_streamer", "undistortImages", 0));
        denoisingMethod->clear();
        denoisingMethod->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_streamer", "NONE", 0)
         << QApplication::translate("MainWindow_streamer", "METHOD X", 0)
        );
        denoisingMethod->setCurrentIndex(0);
        denoising_label->setText(QApplication::translate("MainWindow_streamer", "Denoising", 0));
        normMode_label->setText(QApplication::translate("MainWindow_streamer", "normMode", 0));
        normMode->clear();
        normMode->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_streamer", "FULL_STRETCHING", 0)
         << QApplication::translate("MainWindow_streamer", "EQUALIZATION", 0)
         << QApplication::translate("MainWindow_streamer", "CENTRALIZED", 0)
         << QApplication::translate("MainWindow_streamer", "FIXED_TEMP_RANGE", 0)
         << QApplication::translate("MainWindow_streamer", "FIXED_TEMP_LIMITS", 0)
        );
        normMode->setCurrentIndex(0);
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
        mapCode->setCurrentIndex(0);
        mapCode_label->setText(QApplication::translate("MainWindow_streamer", "mapCode", 0));
        groupBox_Frames->setTitle(QApplication::translate("MainWindow_streamer", "Frame Processing", 0));
        maxReadAttempts->setText(QString());
        framerate_label->setText(QApplication::translate("MainWindow_streamer", "framerate", 0));
        maxReadAttempts_label->setText(QApplication::translate("MainWindow_streamer", "maxReadAttempts", 0));
        framerate->setText(QString());
        groupBox_Comms->setTitle(QApplication::translate("MainWindow_streamer", "Serial Comms", 0));
        serialPollingRate->setText(QString());
        serialPollingRate_label->setText(QApplication::translate("MainWindow_streamer", "serialPollingRate", 0));
        maxNucInterval->setText(QString());
        maxNucThreshold_label->setText(QApplication::translate("MainWindow_streamer", "maxNucThreshold", 0));
        maxNucThreshold->setText(QString());
        maxNucInterval_label->setText(QApplication::translate("MainWindow_streamer", "maxNucInterval", 0));
        groupBox_Input->setTitle(QApplication::translate("MainWindow_streamer", "Input", 0));
        detectorMode->clear();
        detectorMode->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_streamer", "RAW", 0)
         << QApplication::translate("MainWindow_streamer", "LUM", 0)
         << QApplication::translate("MainWindow_streamer", "INS", 0)
         << QApplication::translate("MainWindow_streamer", "RAD", 0)
         << QApplication::translate("MainWindow_streamer", "TMP", 0)
        );
        detectorMode->setCurrentIndex(0);
        detectorMode_label->setText(QApplication::translate("MainWindow_streamer", "detectorMode", 0));
        inputDatatype->clear();
        inputDatatype->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_streamer", "8BIT", 0)
         << QApplication::translate("MainWindow_streamer", "RAW", 0)
         << QApplication::translate("MainWindow_streamer", "MULTIMODAL", 0)
         << QApplication::translate("MainWindow_streamer", "DEPTH", 0)
        );
        inputDatatype->setCurrentIndex(0);
        usbMode->clear();
        usbMode->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_streamer", "USB_16", 0)
         << QApplication::translate("MainWindow_streamer", "USB_8", 0)
        );
        usbMode->setCurrentIndex(0);
        inputDatatype_label->setText(QApplication::translate("MainWindow_streamer", "inputDatatype", 0));
        usbMode_label->setText(QApplication::translate("MainWindow_streamer", "usbMode", 0));
        groupBox_Development->setTitle(QApplication::translate("MainWindow_streamer", "Development", 0));
        debugMode->setText(QApplication::translate("MainWindow_streamer", "debugMode", 0));
        verboseMode->setText(QApplication::translate("MainWindow_streamer", "verboseMode", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow_streamer: public Ui_MainWindow_streamer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_STREAMER_H
