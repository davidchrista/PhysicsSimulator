#-------------------------------------------------
#
# Project created by QtCreator 2016-12-03T15:17:35
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PhysicsSimulator
TEMPLATE = app

CONFIG += c++11


SOURCES += main.cpp\
        mainwindow.cpp \
    rigidbody.cpp \
    negativemassexception.cpp \
    painterwidget.cpp \
    vector.cpp

HEADERS  += mainwindow.h \
    rigidbody.h \
    negativemassexception.h \
    painterwidget.h \
    vector.h \
    timing.h \
    precision.h

FORMS    += mainwindow.ui
