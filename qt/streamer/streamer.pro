#-------------------------------------------------
#
# Project created by QtCreator 2014-07-10T14:41:44
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = streamer
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow_streamer.cpp \
    ../../src/input_stream_config.cpp

HEADERS  += mainwindow_streamer.h \
    ../../include/input_stream_config.hpp

FORMS    += mainwindow_streamer.ui
