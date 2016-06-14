#-------------------------------------------------
#
# Project created by QtCreator 2016-05-24T13:54:50
#
#-------------------------------------------------

QT      += core gui
QT      += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GobotSoftware
TEMPLATE = app


SOURCES += main.cpp\
    Model/map.cpp \
    Controller/mainwindow.cpp \
    Model/robot.cpp \
    View/customqgraphicsview.cpp \
    Model/points.cpp \
    Model/xmlparser.cpp \
    Model/position.cpp \
    Model/group.cpp \
    Model/robots.cpp \
    Model/point.cpp \
    View/robotview.cpp \
    View/mapview.cpp \
    View/pointview.cpp \
    View/bottomlayout.cpp \
    View/pointsleftwidget.cpp \
    View/leftmenuwidget.cpp \
    View/selectedrobotwidget.cpp \
    View/robotsleftwidget.cpp \
    Model/comparerobotsxml.cpp \
    View/mapleftwidget.cpp \
    View/pointsview.cpp \
    View/editselectedrobotwidget.cpp \
    View/robotbtngroup.cpp \
    View/selectedpointwidget.cpp \
    View/editselectedpointwidget.cpp \
    View/groupview.cpp \
    View/groupmenu.cpp \
    Model/pathpoint.cpp \
    View/pathwidget.cpp \
    View/pointlist.cpp \
    View/leftmenu.cpp \
    View/verticalscrollarea.cpp \
    Controller/scanmapthread.cpp \
    Controller/scanmetadatathread.cpp \
    Controller/scanrobotthread.cpp \
    Controller/cmdrobotthread.cpp \
    View/displayselectedpoint.cpp \
    View/pathcreationwidget.cpp \
    View/pointbuttongroup.cpp \
    View/pathpointcreationwidget.cpp \
    View/pathpointlist.cpp \
    View/groupbuttongroup.cpp \
    View/groupeditwindow.cpp \
    View/pathpainter.cpp \
    View/displayselectedgroup.cpp

HEADERS  += Model/map.h \
    Model/position.h \
    Controller/mainwindow.h \
    Model/robot.h \
    View/customqgraphicsview.h \
    Model/points.h \
    Model/xmlparser.h \
    Model/robots.h \
    Model/group.h \
    Model/point.h \
    View/robotview.h \
    View/mapview.h \
    View/pointview.h \
    View/bottomlayout.h \
    View/leftmenuwidget.h \
    View/pointsleftwidget.h \
    View/selectedrobotwidget.h \
    View/robotsleftwidget.h \
    Model/comparerobotsxml.h \
    View/mapleftwidget.h \
    View/pointsview.h \
    View/editselectedrobotwidget.h \
    View/robotbtngroup.h \
    View/selectedpointwidget.h \
    View/editselectedpointwidget.h \
    View/groupview.h \
    View/groupmenu.h \
    Model/pathpoint.h \
    View/pathwidget.h \
    View/pointlist.h \
    View/leftmenu.h \
    View/verticalscrollarea.h \
    Controller/scanmapthread.h \
    Controller/scanmetadatathread.h \
    Controller/scanrobotthread.h \
    Controller/cmdrobotthread.h \
    View/displayselectedpoint.h \
    View/pathcreationwidget.h \
    View/pointbuttongroup.h \
    View/pathpointcreationwidget.h \
    View/pathpointlist.h \
    View/groupbuttongroup.h \
    View/groupeditwindow.h \
    Model/enumgraphicstate.h \
    View/pathpainter.h \
    View/displayselectedgroup.h

FORMS    += Controller/mainwindow.ui

DISTFILES += \
    README \
    Resources/map_ori.png \
    Resources/robot_ori.png

RESOURCES += \
    Resources/resources.qrc \
