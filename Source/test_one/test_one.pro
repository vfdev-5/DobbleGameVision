QT += widgets

OPENCV_DIR = /usr/local/

INCLUDEPATH += $$OPENCV_DIR/include

LIBS += $$OPENCV_DIR/lib/libopencv_core.dylib \
        $$OPENCV_DIR/lib/libopencv_imgproc.dylib \
        $$OPENCV_DIR/lib/libopencv_highgui.dylib \
        $$OPENCV_DIR/lib/libopencv_imgcodecs.dylib

SOURCES += \
    main.cpp \
    ImageFiltering.cpp \
    ImageCommon.cpp \
    Global.cpp

HEADERS += \
    ImageFiltering.h \
    ImageCommon.h \
    Global.h


