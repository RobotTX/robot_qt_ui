######################################################################
# Automatically generated by qmake (2.01a) Fri Jun 24 15:29:08 2016
######################################################################

QT      += core gui
QT      += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GobotSoftware
TEMPLATE = app
DEPENDPATH += . Controller Model Resources View
INCLUDEPATH += . Controller Model View

QMAKE_CXXFLAGS += -std=c++11 -Wall

# Input
HEADERS += Controller/cmdrobotthread.h \
           Controller/mainwindow.h \
           Controller/scanmapthread.h \
           Controller/scanmetadatathread.h \
           Controller/scanrobotthread.h \
           Model/graphicitemstate.h \
           Model/map.h \
           Model/pathpoint.h \
           Model/point.h \
           Model/points.h \
           Model/position.h \
           Model/robot.h \
           Model/robots.h \
           Model/xmlparser.h \
           View/bottomlayout.h \
           View/customqgraphicsview.h \
           View/displayselectedgroup.h \
           View/displayselectedpoint.h \
           View/editselectedrobotwidget.h \
           View/groupbuttongroup.h \
           View/leftmenu.h \
           View/leftmenuwidget.h \
           View/mapleftwidget.h \
           View/mapview.h \
           View/pathcreationwidget.h \
           View/pathpainter.h \
           View/pathpointcreationwidget.h \
           View/pathpointlist.h \
           View/pathwidget.h \
           View/pointbuttongroup.h \
           View/pointsleftwidget.h \
           View/pointview.h \
           View/robotbtngroup.h \
           View/robotsleftwidget.h \
           View/robotview.h \
           View/selectedrobotwidget.h \
           View/spacewidget.h \
           View/toplayout.h \
           View/topleftmenu.h \
           Model/origin.h \
           Controller/updaterobotsthread.h \
           View/createpointwidget.h \
           Controller/sendnewmapthread.h \
           View/customscrollarea.h \
           View/displayselectedpointrobots.h \
           View/displayselectedpath.h \
           View/groupspathswidget.h \
           Model/paths.h \
           View/groupspathsbuttongroup.h \
           View/pathbuttongroup.h \
           View/displaypathgroup.h \
           View/custompushbutton.h \
    View/stylesettings.h \
    View/customlabel.h \
    View/customlineedit.h

FORMS += Controller/mainwindow.ui

SOURCES += main.cpp \
           Controller/cmdrobotthread.cpp \
           Controller/mainwindow.cpp \
           Controller/scanmapthread.cpp \
           Controller/scanmetadatathread.cpp \
           Controller/scanrobotthread.cpp \
           Model/map.cpp \
           Model/pathpoint.cpp \
           Model/point.cpp \
           Model/points.cpp \
           Model/position.cpp \
           Model/robot.cpp \
           Model/robots.cpp \
           Model/xmlparser.cpp \
           View/bottomlayout.cpp \
           View/customqgraphicsview.cpp \
           View/displayselectedgroup.cpp \
           View/displayselectedpoint.cpp \
           View/editselectedrobotwidget.cpp \
           View/groupbuttongroup.cpp \
           View/leftmenu.cpp \
           View/leftmenuwidget.cpp \
           View/mapleftwidget.cpp \
           View/mapview.cpp \
           View/pathcreationwidget.cpp \
           View/pathpainter.cpp \
           View/pathpointcreationwidget.cpp \
           View/pathpointlist.cpp \
           View/pathwidget.cpp \
           View/pointbuttongroup.cpp \
           View/pointsleftwidget.cpp \
           View/pointview.cpp \
           View/robotbtngroup.cpp \
           View/robotsleftwidget.cpp \
           View/robotview.cpp \
           View/selectedrobotwidget.cpp \
           View/spacewidget.cpp \
           View/toplayout.cpp \
           Controller/updaterobotsthread.cpp \
           View/createpointwidget.cpp \
           View/topleftmenu.cpp \
           Controller/sendnewmapthread.cpp \
           View/customscrollarea.cpp \
           View/displayselectedpointrobots.cpp \
           View/displayselectedpath.cpp \
           View/groupspathswidget.cpp \
           Model/paths.cpp \
           View/groupspathsbuttongroup.cpp \
           View/pathbuttongroup.cpp \
           View/displaypathgroup.cpp \
           View/custompushbutton.cpp \
    View/customlabel.cpp \
    View/customlineedit.cpp

RESOURCES += Resources/resources.qrc

DISTFILES += README \
    points.xml \
    Resources/map_ori.png \
    Resources/robot_ori.png \
    Resources/gtrobot-1.pgm \
    Resources/gtrobot-1.pgm.pgm
