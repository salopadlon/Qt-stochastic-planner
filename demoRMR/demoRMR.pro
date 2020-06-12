#-------------------------------------------------
#
# Project created by QtCreator 2018-02-11T14:35:38
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets


BUILDDIR = $$PWD/build

OBJECTS_DIR = $${BUILDDIR}
MOC_DIR = $${BUILDDIR}
RCC_DIR = $${BUILDDIR}
UI_DIR = $${BUILDDIR}

TARGET = demoRMR
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    map_loader.cpp \
    rplidar.cpp \
    CKobuki.cpp \
    rrt.cpp

HEADERS  += mainwindow.h \
    map_loader.h \
    rplidar.h \
    CKobuki.h \
    rrt.h

FORMS    += mainwindow.ui
