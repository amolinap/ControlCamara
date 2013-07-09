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
        #src/ect/QGCLogPlayer.cpp \
        src/ect/MAVLinkProtocol.cc \
        src/ect/QGC.cc \
        #src/ect/MAVLinkSimulationLink.cc \
        src/ect/LinkManager.cc \
        #src/ect/MAVLinkSimulationMAV.cc \
        #src/ect/MAVLinkSimulationWaypointPlanner.cc \
        src/ect/UASManager.cc \
        #src/ect/UAS.cc \
        src/ect/SlugsMAV.cc \
        src/ect/QGCMAVLinkUASFactory.cc \
        src/ect/SerialLink.cc \
        src/ect/qextserialport.cpp \
        src/ect/qextserialbase.cpp \
        src/ect/posix_qextserialport.cpp \
        src/ect/HUD.cc \
        src/ect/UDPLink.cc \
        src/QGCFlight.cpp

HEADERS  += src/mainwindow.h  \
        #src/ect/QGCLogPlayer.h \
        src/ect/MAVLinkProtocol.h \
        src/ect/ProtocolInterface.h \
        src/ect/LinkInterface.h \
        src/ect/QGCMAVLink.h \
        src/ect/QGC.h \
        src/ect/configuration.h \
        #src/ect/MAVLinkSimulationLink.h \
        src/ect/LinkManager.h \
        src/ect/MG.h \
        #src/ect/MAVLinkSimulationMAV.h \
        #src/ect/MAVLinkSimulationWaypointPlanner.h \
        src/ect/UASManager.h \
        #src/ect/UASInterface.h \
        #src/ect/UAS.h \
        src/ect/SlugsMAV.h \
        src/ect/QGCMAVLinkUASFactory.h \
        src/ect/SerialLink.h \
        src/ect/qextserialport.h \
        src/ect/qextserialbase.h \
        src/ect/SerialLinkInterface.h \
        src/ect/posix_qextserialport.h \
        src/ect/HUD.h \
        src/ect/UDPLink.h \
        src/QGCFlight.h

FORMS    += src/mainwindow.ui  \
        #src/ect/QGCLogPlayer.ui \
    src/QGCFlight.ui

unix:DEFINES           += _TTY_POSIX_
