# -------------------------------------------------
# Project created by QtCreator 2014-03-18T10:40:12
# -------------------------------------------------
TARGET = QT_Demo
TEMPLATE = app

INCLUDEPATH += "../../include/"
LIBS += "../../lib/gcc-4.4.3/libMVSDK.so"

SOURCES += main.cpp \
    mainwindow.cpp \
    capturethread.cpp
HEADERS += mainwindow.h \
    capturethread.h
FORMS += mainwindow.ui
