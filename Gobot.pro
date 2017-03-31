QT += qml quick
QT += widgets

CONFIG += c++11

SOURCES += main.cpp \
    Controller/maincontroller.cpp \
    Helper/helper.cpp \
    Model/Map/map.cpp \
    Controller/Map/mapcontroller.cpp \
    Model/Point/xmlparser.cpp \
    Model/Point/point.cpp \
    Model/Point/points.cpp \
    Controller/Point/pointcontroller.cpp \
    Controller/Path/pathcontroller.cpp \
    Model/Path/paths.cpp \
    Model/Path/pathpoint.cpp \
    Model/Path/path.cpp \
    Model/Path/pathgroup.cpp \
    Model/Point/pointgroup.cpp \
    Model/Path/pathxmlparser.cpp \
    View/editmappainteditem.cpp \
    Controller/Map/editmapcontroller.cpp \
    Controller/Robot/robotcontroller.cpp \
    Controller/Robot/robotscontroller.cpp \
    Controller/Robot/robotserverworker.cpp \
    Controller/Robot/cmdrobotworker.cpp \
    Controller/Robot/robotpositionworker.cpp \
    Controller/Robot/teleopworker.cpp \
    Controller/Map/localmapworker.cpp \
    Controller/Map/metadataworker.cpp \
    Controller/Map/particlecloudworker.cpp \
    Controller/Map/scanmapworker.cpp \
    Controller/Map/sendnewmapworker.cpp \
    Controller/Map/mergemapcontroller.cpp \
    Controller/Robot/commandcontroller.cpp \
    View/mergemapspainteditem.cpp

RESOURCES += qml.qrc \
    Resources/resources.qrc

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH =

# Additional import path used to resolve QML modules just for Qt Quick Designer
QML_DESIGNER_IMPORT_PATH =

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    Controller/maincontroller.h \
    Helper/helper.h \
    Model/Map/map.h \
    Controller/Map/mapcontroller.h \
    Model/Point/xmlparser.h \
    Model/Point/point.h \
    Model/Point/points.h \
    Controller/Point/pointcontroller.h \
    Controller/Path/pathcontroller.h \
    Model/Path/paths.h \
    Model/Path/pathpoint.h \
    Model/Path/path.h \
    Model/Path/pathgroup.h \
    Model/Point/pointgroup.h \
    Model/Path/pathxmlparser.h \
    View/editmappainteditem.h \
    Controller/Map/editmapcontroller.h \
    Controller/Robot/robotcontroller.h \
    Controller/Robot/robotscontroller.h \
    Controller/Robot/robotserverworker.h \
    Controller/Robot/cmdrobotworker.h \
    Controller/Robot/robotpositionworker.h \
    Controller/Robot/teleopworker.h \
    Controller/Map/localmapworker.h \
    Controller/Map/metadataworker.h \
    Controller/Map/particlecloudworker.h \
    Controller/Map/scanmapworker.h \
    Controller/Map/sendnewmapworker.h \
    Controller/Map/mergemapcontroller.h \
    Controller/Robot/commandcontroller.h \
    View/mergemapspainteditem.h
