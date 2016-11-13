#-------------------------------------------------
#
# Project created by QtCreator 2016-10-13T23:23:26
#
#-------------------------------------------------

QT       += core gui
QT += widgets serialport
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = OPENCVshow
TEMPLATE = app


SOURCES += main.cpp\
           mainwindow.cpp \
           opencvdeal.cpp \
           Vibe.cpp \
           serialconnect.cpp

HEADERS  += mainwindow.h \
            opencvdeal.h \
            Vibe.h \
            serialconnect.h

FORMS    += mainwindow.ui

INCLUDEPATH +=.
LIBS += -Lopencv_ffmpeg2412_64.dll

INCLUDEPATH += E:\opencv\opencv\build\include
#INCLUDEPATH +=E:\opencv\opencv\sources\modules\highgui\src

win32:CONFIG(debug, debug|release): {
LIBS += -LE:\opencv\opencv\build\x64\vc12\lib \
-lopencv_objdetect2412d\
-lopencv_ts2412d\
-lopencv_video2412d\
-lopencv_nonfree2412d\
-lopencv_ocl2412d\
-lopencv_photo2412d\
-lopencv_stitching2412d\
-lopencv_superres2412d\
-lopencv_videostab2412d\
-lopencv_calib3d2412d\
-lopencv_contrib2412d\
-lopencv_core2412d\
-lopencv_features2d2412d\
-lopencv_flann2412d\
-lopencv_gpu2412d\
-lopencv_highgui2412d\
-lopencv_imgproc2412d\
-lopencv_legacy2412d\
-lopencv_ml2412
} else:win32:CONFIG(release, debug|release): {
LIBS += -LE:\opencv\opencv\build\x64\vc12\lib \
-lopencv_objdetect2412 \
-lopencv_ts2412 \
-lopencv_video2412 \
-lopencv_nonfree2412 \
-lopencv_ocl2412 \
-lopencv_photo2412 \
-lopencv_stitching2412 \
-lopencv_superres2412 \
-lopencv_videostab2412 \
-lopencv_calib3d2412 \
-lopencv_contrib2412 \
-lopencv_core2412 \
-lopencv_features2d2412 \
-lopencv_flann2412 \
-lopencv_gpu2412 \
-lopencv_highgui2412 \
-lopencv_imgproc2412 \
-lopencv_legacy2412 \
-lopencv_ml2412
}

DISTFILES += \
    4.avi \
    3.avi
