#-------------------------------------------------
#
# Project created by QtCreator 2018-09-13T00:00:30
#
#-------------------------------------------------

QT       += core gui widgets printsupport

TARGET = DOB_axis_1
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11
CONFIG += console

SOURCES += \
        main.cpp \
        dob_axis_1.cpp \
    qcustomplot.cpp \
    DxlControl.cpp

HEADERS += \
        dob_axis_1.h \
    qcustomplot.h \
    DxlControl.h

FORMS += \
        dob_axis_1.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32: LIBS += -L$$PWD/lib/ -ldxl_x64_cpp

INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD/include

win32:!win32-g++: PRE_TARGETDEPS += $$PWD/lib/dxl_x64_cpp.lib
else:win32-g++: PRE_TARGETDEPS += $$PWD/lib/libdxl_x64_cpp.a
