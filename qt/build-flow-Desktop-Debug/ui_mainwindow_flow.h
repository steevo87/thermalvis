/********************************************************************************
** Form generated from reading UI file 'mainwindow_flow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_FLOW_H
#define UI_MAINWINDOW_FLOW_H

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

class Ui_MainWindow_flow
{
public:
    QWidget *centralWidget;
    QCheckBox *debugMode;
    QCheckBox *verboseMode;
    QCheckBox *showTrackHistory;
    QCheckBox *adaptiveWindow;
    QCheckBox *velocityPrediction;
    QCheckBox *attemptHistoricalRecovery;
    QCheckBox *autoTrackManagement;
    QCheckBox *attemptMatching;
    QCheckBox *detectEveryFrame;
    QLabel *maxFeatures_label;
    QLineEdit *maxFeatures;
    QLabel *minFeaturesLabel;
    QLineEdit *minFeatures;
    QLineEdit *drawingHistory;
    QLabel *drawingHistory_label;
    QComboBox *matchingMode;
    QLineEdit *maxFrac;
    QLabel *maxFrac_label;
    QLineEdit *minSeparation;
    QLabel *minSeparation_label;
    QLabel *maxVelocity_label;
    QLineEdit *maxVelocity;
    QLineEdit *newFeaturesPeriod;
    QLabel *newFeaturesPeriod_label;
    QLineEdit *delayTimeout;
    QLabel *delayTimeout_label;
    QLineEdit *sensitivity_1;
    QLineEdit *sensitivity_2;
    QLineEdit *sensitivity_3;
    QLabel *sensitivity_1_label;
    QLabel *sensitivity_2_label;
    QLabel *sensitivity_3_label;
    QLineEdit *multiplier_1;
    QLabel *multiplier_1_label;
    QLabel *multiplier_2_label;
    QLineEdit *multiplier_2;
    QComboBox *detector_1;
    QComboBox *detector_2;
    QComboBox *detector_3;
    QLabel *detector_1_label;
    QLabel *detector_2_label;
    QLabel *detector_3_label;
    QLabel *matchingMode_label;
    QLineEdit *flowThreshold;
    QLabel *flowThreshold_label;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow_flow)
    {
        if (MainWindow_flow->objectName().isEmpty())
            MainWindow_flow->setObjectName(QString::fromUtf8("MainWindow_flow"));
        MainWindow_flow->resize(640, 320);
        centralWidget = new QWidget(MainWindow_flow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        debugMode = new QCheckBox(centralWidget);
        debugMode->setObjectName(QString::fromUtf8("debugMode"));
        debugMode->setGeometry(QRect(10, 10, 81, 17));
        verboseMode = new QCheckBox(centralWidget);
        verboseMode->setObjectName(QString::fromUtf8("verboseMode"));
        verboseMode->setGeometry(QRect(10, 30, 91, 17));
        showTrackHistory = new QCheckBox(centralWidget);
        showTrackHistory->setObjectName(QString::fromUtf8("showTrackHistory"));
        showTrackHistory->setGeometry(QRect(10, 50, 111, 17));
        adaptiveWindow = new QCheckBox(centralWidget);
        adaptiveWindow->setObjectName(QString::fromUtf8("adaptiveWindow"));
        adaptiveWindow->setGeometry(QRect(10, 80, 111, 17));
        velocityPrediction = new QCheckBox(centralWidget);
        velocityPrediction->setObjectName(QString::fromUtf8("velocityPrediction"));
        velocityPrediction->setGeometry(QRect(10, 110, 111, 17));
        attemptHistoricalRecovery = new QCheckBox(centralWidget);
        attemptHistoricalRecovery->setObjectName(QString::fromUtf8("attemptHistoricalRecovery"));
        attemptHistoricalRecovery->setGeometry(QRect(10, 160, 151, 17));
        autoTrackManagement = new QCheckBox(centralWidget);
        autoTrackManagement->setObjectName(QString::fromUtf8("autoTrackManagement"));
        autoTrackManagement->setGeometry(QRect(10, 140, 131, 17));
        attemptMatching = new QCheckBox(centralWidget);
        attemptMatching->setObjectName(QString::fromUtf8("attemptMatching"));
        attemptMatching->setGeometry(QRect(10, 180, 111, 17));
        detectEveryFrame = new QCheckBox(centralWidget);
        detectEveryFrame->setObjectName(QString::fromUtf8("detectEveryFrame"));
        detectEveryFrame->setGeometry(QRect(510, 140, 111, 17));
        maxFeatures_label = new QLabel(centralWidget);
        maxFeatures_label->setObjectName(QString::fromUtf8("maxFeatures_label"));
        maxFeatures_label->setGeometry(QRect(470, 40, 91, 16));
        maxFeatures_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxFeatures = new QLineEdit(centralWidget);
        maxFeatures->setObjectName(QString::fromUtf8("maxFeatures"));
        maxFeatures->setGeometry(QRect(570, 40, 51, 20));
        minFeaturesLabel = new QLabel(centralWidget);
        minFeaturesLabel->setObjectName(QString::fromUtf8("minFeaturesLabel"));
        minFeaturesLabel->setGeometry(QRect(470, 10, 91, 16));
        minFeaturesLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        minFeatures = new QLineEdit(centralWidget);
        minFeatures->setObjectName(QString::fromUtf8("minFeatures"));
        minFeatures->setGeometry(QRect(570, 10, 51, 20));
        drawingHistory = new QLineEdit(centralWidget);
        drawingHistory->setObjectName(QString::fromUtf8("drawingHistory"));
        drawingHistory->setGeometry(QRect(210, 10, 51, 20));
        drawingHistory_label = new QLabel(centralWidget);
        drawingHistory_label->setObjectName(QString::fromUtf8("drawingHistory_label"));
        drawingHistory_label->setGeometry(QRect(110, 10, 91, 16));
        drawingHistory_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        matchingMode = new QComboBox(centralWidget);
        matchingMode->setObjectName(QString::fromUtf8("matchingMode"));
        matchingMode->setGeometry(QRect(370, 10, 91, 22));
        maxFrac = new QLineEdit(centralWidget);
        maxFrac->setObjectName(QString::fromUtf8("maxFrac"));
        maxFrac->setGeometry(QRect(320, 70, 61, 20));
        maxFrac_label = new QLabel(centralWidget);
        maxFrac_label->setObjectName(QString::fromUtf8("maxFrac_label"));
        maxFrac_label->setGeometry(QRect(220, 70, 91, 16));
        maxFrac_label->setFrameShape(QFrame::NoFrame);
        maxFrac_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        minSeparation = new QLineEdit(centralWidget);
        minSeparation->setObjectName(QString::fromUtf8("minSeparation"));
        minSeparation->setGeometry(QRect(570, 70, 51, 20));
        minSeparation_label = new QLabel(centralWidget);
        minSeparation_label->setObjectName(QString::fromUtf8("minSeparation_label"));
        minSeparation_label->setGeometry(QRect(470, 70, 91, 16));
        minSeparation_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxVelocity_label = new QLabel(centralWidget);
        maxVelocity_label->setObjectName(QString::fromUtf8("maxVelocity_label"));
        maxVelocity_label->setGeometry(QRect(220, 100, 91, 16));
        maxVelocity_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        maxVelocity = new QLineEdit(centralWidget);
        maxVelocity->setObjectName(QString::fromUtf8("maxVelocity"));
        maxVelocity->setGeometry(QRect(320, 100, 61, 20));
        newFeaturesPeriod = new QLineEdit(centralWidget);
        newFeaturesPeriod->setObjectName(QString::fromUtf8("newFeaturesPeriod"));
        newFeaturesPeriod->setGeometry(QRect(570, 100, 51, 20));
        newFeaturesPeriod_label = new QLabel(centralWidget);
        newFeaturesPeriod_label->setObjectName(QString::fromUtf8("newFeaturesPeriod_label"));
        newFeaturesPeriod_label->setGeometry(QRect(450, 100, 111, 20));
        newFeaturesPeriod_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        delayTimeout = new QLineEdit(centralWidget);
        delayTimeout->setObjectName(QString::fromUtf8("delayTimeout"));
        delayTimeout->setGeometry(QRect(220, 220, 51, 20));
        delayTimeout_label = new QLabel(centralWidget);
        delayTimeout_label->setObjectName(QString::fromUtf8("delayTimeout_label"));
        delayTimeout_label->setGeometry(QRect(140, 220, 71, 20));
        delayTimeout_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        sensitivity_1 = new QLineEdit(centralWidget);
        sensitivity_1->setObjectName(QString::fromUtf8("sensitivity_1"));
        sensitivity_1->setGeometry(QRect(552, 180, 71, 20));
        sensitivity_2 = new QLineEdit(centralWidget);
        sensitivity_2->setObjectName(QString::fromUtf8("sensitivity_2"));
        sensitivity_2->setGeometry(QRect(552, 210, 71, 20));
        sensitivity_3 = new QLineEdit(centralWidget);
        sensitivity_3->setObjectName(QString::fromUtf8("sensitivity_3"));
        sensitivity_3->setGeometry(QRect(552, 240, 71, 20));
        sensitivity_1_label = new QLabel(centralWidget);
        sensitivity_1_label->setObjectName(QString::fromUtf8("sensitivity_1_label"));
        sensitivity_1_label->setGeometry(QRect(450, 180, 91, 16));
        sensitivity_1_label->setFrameShape(QFrame::NoFrame);
        sensitivity_1_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        sensitivity_2_label = new QLabel(centralWidget);
        sensitivity_2_label->setObjectName(QString::fromUtf8("sensitivity_2_label"));
        sensitivity_2_label->setGeometry(QRect(450, 210, 91, 16));
        sensitivity_2_label->setFrameShape(QFrame::NoFrame);
        sensitivity_2_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        sensitivity_3_label = new QLabel(centralWidget);
        sensitivity_3_label->setObjectName(QString::fromUtf8("sensitivity_3_label"));
        sensitivity_3_label->setGeometry(QRect(450, 240, 91, 16));
        sensitivity_3_label->setFrameShape(QFrame::NoFrame);
        sensitivity_3_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        multiplier_1 = new QLineEdit(centralWidget);
        multiplier_1->setObjectName(QString::fromUtf8("multiplier_1"));
        multiplier_1->setGeometry(QRect(80, 210, 41, 20));
        multiplier_1_label = new QLabel(centralWidget);
        multiplier_1_label->setObjectName(QString::fromUtf8("multiplier_1_label"));
        multiplier_1_label->setGeometry(QRect(10, 210, 61, 20));
        multiplier_1_label->setFrameShape(QFrame::NoFrame);
        multiplier_1_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        multiplier_2_label = new QLabel(centralWidget);
        multiplier_2_label->setObjectName(QString::fromUtf8("multiplier_2_label"));
        multiplier_2_label->setGeometry(QRect(10, 240, 61, 20));
        multiplier_2_label->setFrameShape(QFrame::NoFrame);
        multiplier_2_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        multiplier_2 = new QLineEdit(centralWidget);
        multiplier_2->setObjectName(QString::fromUtf8("multiplier_2"));
        multiplier_2->setGeometry(QRect(80, 240, 41, 20));
        detector_1 = new QComboBox(centralWidget);
        detector_1->setObjectName(QString::fromUtf8("detector_1"));
        detector_1->setGeometry(QRect(400, 180, 51, 22));
        detector_2 = new QComboBox(centralWidget);
        detector_2->setObjectName(QString::fromUtf8("detector_2"));
        detector_2->setGeometry(QRect(400, 210, 51, 22));
        detector_3 = new QComboBox(centralWidget);
        detector_3->setObjectName(QString::fromUtf8("detector_3"));
        detector_3->setGeometry(QRect(400, 240, 51, 22));
        detector_1_label = new QLabel(centralWidget);
        detector_1_label->setObjectName(QString::fromUtf8("detector_1_label"));
        detector_1_label->setGeometry(QRect(330, 180, 61, 20));
        detector_1_label->setFrameShape(QFrame::NoFrame);
        detector_1_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        detector_2_label = new QLabel(centralWidget);
        detector_2_label->setObjectName(QString::fromUtf8("detector_2_label"));
        detector_2_label->setGeometry(QRect(330, 210, 61, 20));
        detector_2_label->setFrameShape(QFrame::NoFrame);
        detector_2_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        detector_3_label = new QLabel(centralWidget);
        detector_3_label->setObjectName(QString::fromUtf8("detector_3_label"));
        detector_3_label->setGeometry(QRect(330, 240, 61, 20));
        detector_3_label->setFrameShape(QFrame::NoFrame);
        detector_3_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        matchingMode_label = new QLabel(centralWidget);
        matchingMode_label->setObjectName(QString::fromUtf8("matchingMode_label"));
        matchingMode_label->setGeometry(QRect(290, 10, 71, 20));
        matchingMode_label->setFrameShape(QFrame::NoFrame);
        matchingMode_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        flowThreshold = new QLineEdit(centralWidget);
        flowThreshold->setObjectName(QString::fromUtf8("flowThreshold"));
        flowThreshold->setGeometry(QRect(320, 130, 61, 20));
        flowThreshold_label = new QLabel(centralWidget);
        flowThreshold_label->setObjectName(QString::fromUtf8("flowThreshold_label"));
        flowThreshold_label->setGeometry(QRect(220, 130, 91, 16));
        flowThreshold_label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        MainWindow_flow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow_flow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 640, 21));
        MainWindow_flow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow_flow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow_flow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow_flow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow_flow->setStatusBar(statusBar);

        retranslateUi(MainWindow_flow);

        matchingMode->setCurrentIndex(2);
        detector_1->setCurrentIndex(2);
        detector_2->setCurrentIndex(0);
        detector_3->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow_flow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow_flow)
    {
        MainWindow_flow->setWindowTitle(QApplication::translate("MainWindow_flow", "MainWindow_flow", 0));
        debugMode->setText(QApplication::translate("MainWindow_flow", "debugMode", 0));
        verboseMode->setText(QApplication::translate("MainWindow_flow", "verboseMode", 0));
        showTrackHistory->setText(QApplication::translate("MainWindow_flow", "showTrackHistory", 0));
        adaptiveWindow->setText(QApplication::translate("MainWindow_flow", "adaptiveWindow", 0));
        velocityPrediction->setText(QApplication::translate("MainWindow_flow", "velocityPrediction", 0));
        attemptHistoricalRecovery->setText(QApplication::translate("MainWindow_flow", "attemptHistoricalRecovery", 0));
        autoTrackManagement->setText(QApplication::translate("MainWindow_flow", "autoTrackManagement", 0));
        attemptMatching->setText(QApplication::translate("MainWindow_flow", "attemptMatching", 0));
        detectEveryFrame->setText(QApplication::translate("MainWindow_flow", "detectEveryFrame", 0));
        maxFeatures_label->setText(QApplication::translate("MainWindow_flow", "maxFeatures", 0));
        maxFeatures->setText(QString());
        minFeaturesLabel->setText(QApplication::translate("MainWindow_flow", "minFeatures", 0));
        minFeatures->setText(QString());
        drawingHistory->setText(QString());
        drawingHistory_label->setText(QApplication::translate("MainWindow_flow", "drawingHistory", 0));
        matchingMode->clear();
        matchingMode->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_flow", "NEAREST NEIGHBOUR", 0)
         << QApplication::translate("MainWindow_flow", "NN DISTANCE RATIO", 0)
         << QApplication::translate("MainWindow_flow", "NN/NNDR LDA", 0)
        );
        matchingMode->setCurrentIndex(matchingMode->findText("NN/NNDR LDA"));
        maxFrac->setText(QString());
        maxFrac_label->setText(QApplication::translate("MainWindow_flow", "maxFrac", 0));
        minSeparation->setText(QString());
        minSeparation_label->setText(QApplication::translate("MainWindow_flow", "minSeparation", 0));
        maxVelocity_label->setText(QApplication::translate("MainWindow_flow", "maxVelocity", 0));
        maxVelocity->setText(QString());
        newFeaturesPeriod->setText(QString());
        newFeaturesPeriod_label->setText(QApplication::translate("MainWindow_flow", "newFeaturesPeriod", 0));
        delayTimeout->setText(QString());
        delayTimeout_label->setText(QApplication::translate("MainWindow_flow", "delayTimeout", 0));
        sensitivity_1->setText(QString());
        sensitivity_2->setText(QString());
        sensitivity_3->setText(QString());
        sensitivity_1_label->setText(QApplication::translate("MainWindow_flow", "sensitivity_1", 0));
        sensitivity_2_label->setText(QApplication::translate("MainWindow_flow", "sensitivity_2", 0));
        sensitivity_3_label->setText(QApplication::translate("MainWindow_flow", "sensitivity_3", 0));
        multiplier_1->setText(QString());
        multiplier_1_label->setText(QApplication::translate("MainWindow_flow", "multiplier_1", 0));
        multiplier_2_label->setText(QApplication::translate("MainWindow_flow", "multiplier_2", 0));
        multiplier_2->setText(QString());
        detector_1->clear();
        detector_1->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_flow", "OFF", 0)
         << QApplication::translate("MainWindow_flow", "GFTT", 0)
         << QApplication::translate("MainWindow_flow", "FAST", 0)
         << QApplication::translate("MainWindow_flow", "HARRIS", 0)
         << QApplication::translate("MainWindow_flow", "FILE", 0)
        );
        detector_1->setCurrentIndex(detector_1->findText("FAST"));
        detector_2->clear();
        detector_2->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_flow", "OFF", 0)
         << QApplication::translate("MainWindow_flow", "GFTT", 0)
         << QApplication::translate("MainWindow_flow", "FAST", 0)
         << QApplication::translate("MainWindow_flow", "HARRIS", 0)
         << QApplication::translate("MainWindow_flow", "FILE", 0)
        );
        detector_2->setCurrentIndex(detector_2->findText("OFF"));
        detector_3->clear();
        detector_3->insertItems(0, QStringList()
         << QApplication::translate("MainWindow_flow", "OFF", 0)
         << QApplication::translate("MainWindow_flow", "GFTT", 0)
         << QApplication::translate("MainWindow_flow", "FAST", 0)
         << QApplication::translate("MainWindow_flow", "HARRIS", 0)
         << QApplication::translate("MainWindow_flow", "FILE", 0)
        );
        detector_3->setCurrentIndex(detector_3->findText("OFF"));
        detector_1_label->setText(QApplication::translate("MainWindow_flow", "detector_1", 0));
        detector_2_label->setText(QApplication::translate("MainWindow_flow", "detector_2", 0));
        detector_3_label->setText(QApplication::translate("MainWindow_flow", "detector_3", 0));
        matchingMode_label->setText(QApplication::translate("MainWindow_flow", "matchingMode", 0));
        flowThreshold->setText(QString());
        flowThreshold_label->setText(QApplication::translate("MainWindow_flow", "flowThreshold", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow_flow: public Ui_MainWindow_flow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_FLOW_H
