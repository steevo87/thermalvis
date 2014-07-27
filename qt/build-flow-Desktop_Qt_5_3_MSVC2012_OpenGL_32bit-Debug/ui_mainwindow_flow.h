/********************************************************************************
** Form generated from reading UI file 'mainwindow_flow.ui'
**
** Created by: Qt User Interface Compiler version 5.3.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_FLOW_H
#define UI_MAINWINDOW_FLOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

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
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow_flow)
    {
        if (MainWindow_flow->objectName().isEmpty())
            MainWindow_flow->setObjectName(QStringLiteral("MainWindow_flow"));
        MainWindow_flow->resize(640, 320);
        centralWidget = new QWidget(MainWindow_flow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        debugMode = new QCheckBox(centralWidget);
        debugMode->setObjectName(QStringLiteral("debugMode"));
        debugMode->setGeometry(QRect(10, 10, 81, 17));
        verboseMode = new QCheckBox(centralWidget);
        verboseMode->setObjectName(QStringLiteral("verboseMode"));
        verboseMode->setGeometry(QRect(10, 30, 91, 17));
        showTrackHistory = new QCheckBox(centralWidget);
        showTrackHistory->setObjectName(QStringLiteral("showTrackHistory"));
        showTrackHistory->setGeometry(QRect(10, 50, 111, 17));
        adaptiveWindow = new QCheckBox(centralWidget);
        adaptiveWindow->setObjectName(QStringLiteral("adaptiveWindow"));
        adaptiveWindow->setGeometry(QRect(140, 10, 111, 17));
        velocityPrediction = new QCheckBox(centralWidget);
        velocityPrediction->setObjectName(QStringLiteral("velocityPrediction"));
        velocityPrediction->setGeometry(QRect(140, 30, 111, 17));
        attemptHistoricalRecovery = new QCheckBox(centralWidget);
        attemptHistoricalRecovery->setObjectName(QStringLiteral("attemptHistoricalRecovery"));
        attemptHistoricalRecovery->setGeometry(QRect(140, 50, 151, 17));
        autoTrackManagement = new QCheckBox(centralWidget);
        autoTrackManagement->setObjectName(QStringLiteral("autoTrackManagement"));
        autoTrackManagement->setGeometry(QRect(310, 10, 131, 17));
        attemptMatching = new QCheckBox(centralWidget);
        attemptMatching->setObjectName(QStringLiteral("attemptMatching"));
        attemptMatching->setGeometry(QRect(310, 30, 111, 17));
        detectEveryFrame = new QCheckBox(centralWidget);
        detectEveryFrame->setObjectName(QStringLiteral("detectEveryFrame"));
        detectEveryFrame->setGeometry(QRect(310, 50, 111, 17));
        MainWindow_flow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow_flow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 640, 21));
        MainWindow_flow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow_flow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow_flow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow_flow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow_flow->setStatusBar(statusBar);

        retranslateUi(MainWindow_flow);

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
    } // retranslateUi

};

namespace Ui {
    class MainWindow_flow: public Ui_MainWindow_flow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_FLOW_H
