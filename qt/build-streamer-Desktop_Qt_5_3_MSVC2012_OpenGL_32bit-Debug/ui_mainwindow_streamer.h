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
#include <QtWidgets/QHeaderView>
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

        QMetaObject::connectSlotsByName(MainWindow_streamer);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow_streamer)
    {
        MainWindow_streamer->setWindowTitle(QApplication::translate("MainWindow_streamer", "MainWindow_streamer", 0));
        debugMode->setText(QApplication::translate("MainWindow_streamer", "debugMode", 0));
        verboseMode->setText(QApplication::translate("MainWindow_streamer", "verboseMode", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow_streamer: public Ui_MainWindow_streamer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_STREAMER_H
