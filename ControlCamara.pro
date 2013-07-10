#-------------------------------------------------
#
# Project created by QtCreator 2013-06-18T13:47:17
#
#-------------------------------------------------

QT       += network core gui opengl webkit

TARGET = ControlCamara
TEMPLATE = app

BASEDIR = $$IN_PWD
TARGETDIR = $$OUT_PWD
BUILDDIR = $$TARGETDIR/build
LANGUAGE = C++
OBJECTS_DIR = $$BUILDDIR/obj
MOC_DIR = $$BUILDDIR/moc
UI_HEADERS_DIR = src/ui/generated

ICON = icon.icns

FLIGHT_SOURCE = src/

INCLUDEPATH += $$FLIGHT_SOURCE \
            $$FLIGHT_SOURCE/ect \
            $$BASEDIR/../../mavlink/include/gimbal \
            $$BASEDIR/../../mavlink/include

SOURCES += src/main.cpp \
        src/mainwindow.cpp \        
        src/ect/MAVLinkProtocol.cc \
        src/ect/QGC.cc \
        src/ect/LinkManager.cc \
        src/ect/UASManager.cc \
        src/ect/SlugsMAV.cc \
        src/ect/QGCMAVLinkUASFactory.cc \
        src/ect/qextserialport.cpp \
        src/ect/qextserialbase.cpp \
        src/ect/posix_qextserialport.cpp \
        src/ect/UDPLink.cc

HEADERS  += src/mainwindow.h  \
        src/ect/MAVLinkProtocol.h \
        src/ect/ProtocolInterface.h \
        src/ect/LinkInterface.h \
        src/ect/QGCMAVLink.h \
        src/ect/QGC.h \
        src/ect/configuration.h \
        src/ect/LinkManager.h \
        src/ect/MG.h \
        src/ect/UASManager.h \
        src/ect/SlugsMAV.h \
        src/ect/QGCMAVLinkUASFactory.h \
        src/ect/qextserialport.h \
        src/ect/qextserialbase.h \
        src/ect/posix_qextserialport.h \
        src/ect/UDPLink.h

FORMS    += src/mainwindow.ui

unix:DEFINES           += _TTY_POSIX_

QMAKE_POST_LINK += echo "Copying files"
QMAKE_POST_LINK += && cp -f $$BASEDIR/images/style-mission.css $$TARGETDIR/ControlCamara.app/Contents/MacOS/style-indoor.css

RESOURCES += \
    resources.qrc
