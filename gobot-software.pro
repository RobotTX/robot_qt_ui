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
HEADERS += \
           Controller/mainwindow.h \
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
           View/spacewidget.h \
           View/toplayout.h \
           View/topleftmenu.h \
           View/createpointwidget.h \
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
           View/customlineedit.h \
           View/customrobotdialog.h \
           Controller/cmdrobotworker.h \
           Controller/metadataworker.h \
           Controller/robotpositionworker.h \
           Controller/sendnewmapworker.h \
           Controller/scanmapworker.h \
           Controller/commandcontroller.h \
           View/commandmessagebox.h \
           zconf.h \
           zipreader.h \
           zipwriter.h \
           zlib.h \
           Controller/robotserverworker.h \
           View/editselectedrobotwidget.h \
    View/editmapwidget.h \
    View/editmapview.h \
    Controller/localmapworker.h

FORMS += Controller/mainwindow.ui

SOURCES += main.cpp \
           Controller/mainwindow.cpp \
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
           View/spacewidget.cpp \
           View/toplayout.cpp \
           View/createpointwidget.cpp \
           View/topleftmenu.cpp \
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
           View/customlineedit.cpp \
           View/customrobotdialog.cpp \
           Controller/cmdrobotworker.cpp \
           Controller/robotpositionworker.cpp \
           Controller/metadataworker.cpp \
           Controller/sendnewmapworker.cpp \
           Controller/scanmapworker.cpp \
           Controller/commandcontroller.cpp \
           View/commandmessagebox.cpp \
           zip.cpp \
           Controller/robotserverworker.cpp \
           View/editselectedrobotwidget.cpp \
    View/editmapwidget.cpp \
    View/editmapview.cpp \
    Controller/localmapworker.cpp


RESOURCES += Resources/resources.qrc

DISTFILES += README \
    points.xml \
    Resources/map_ori.png \
    Resources/robot_ori.png \
    Resources/gtrobot-1.pgm \
    Resources/gtrobot-1.pgm.pgm

ZLIBCODEDIR = "$$PWD/QtZipTest/zlib"

INCLUDEPATH += $$ZLIBCODEDIR

unix {
    LIBS += -L$${ZLIBCODEDIR} -lz
}

win32 {
    LIBS += -L$${ZLIBCODEDIR}/Windows -lzdll
}


