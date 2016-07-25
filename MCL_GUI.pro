#-------------------------------------------------
#
# Project created by QtCreator 2016-05-04T12:26:59
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = MCL_GUI
TEMPLATE = app
CONFIG += c++11

SOURCES += main.cpp\
        mainwindow.cpp \
    mcl.cpp \
    robot.cpp \
    thread_mcl.cpp \
    map.cpp \
    kld.cpp

HEADERS  += mainwindow.h \
    mcl.h \
    robot.h \
    thread_mcl.h \
    map.h \
    kld.h

FORMS    += mainwindow.ui
