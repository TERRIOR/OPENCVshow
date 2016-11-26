#-------------------------------------------------
#
# Project created by QtCreator 2016-10-13T23:23:26
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = OPENCVshow
TEMPLATE = app


SOURCES += main.cpp\
           mainwindow.cpp \
           opencvdeal.cpp \
           Vibe.cpp \
           piserialconnect.cpp

HEADERS  += mainwindow.h \
            opencvdeal.h \
            Vibe.h \
            piserialconnect.h

FORMS    += mainwindow.ui



INCLUDEPATH += /usr/local/include/opencv
INCLUDEPATH += /usr/local/include/opencv2
LIBS +=-lwiringPi
LIBS += /usr/local/lib/libopencv_calib3d.so.2.4.13\
/usr/local/lib/libopencv_core.so.2.4.13\
/usr/local/lib/libopencv_highgui.so.2.4.13\
/usr/local/lib/libopencv_imgproc.so.2.4.13\
/usr/local/lib/libopencv_contrib.so.2.4.13\
/usr/local/lib/libopencv_video.so.2.4.13\
/usr/local/lib/libopencv_objdetect.so.2.4.13\
/usr/local/lib/libopencv_features2d.so.2.4.13
DISTFILES += \
    4.avi \
    3.avi
