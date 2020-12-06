QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += main.cpp \
    anglesolver.cpp \
    armordetectpr.cpp \
    rmvideocapture.cpp \
    anti_top.cpp \
    system_time.cpp \
    armor_finder.cpp \
    classifier_num.cpp \
    crc_check.cpp \
    serialport.cpp

HEADERS += \
    anglesolver.hpp \
    armordetectpr.hpp \
    rmvideocapture.h \
    roundqueue.h \
    system_time.h \
    armor_finder.h \
    classifier_num.h \
    cameraapi.h \
    cameradefine.h \
    camerastatus.h \
    crc_check.h \
    serialport.h

INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv \
                /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_*.so
