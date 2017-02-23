######################################################################
# Automatically generated by qmake (2.01a) Fri Jun 24 15:29:08 2016
######################################################################

QT      += core gui
QT      += network
QT      += multimedia
QT      += multimediawidgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GobotSoftware
TEMPLATE = app
DEPENDPATH += . Controller Model Resources View
INCLUDEPATH += . Controller Model View

QMAKE_CXXFLAGS += -std=c++11 -Wall -lz

DEFINES *= QT_USE_QSTRINGBUILDER

# Input
HEADERS += \
    Controller/Map/localmapworker.h \
    Controller/Map/mapcontroller.h \
    Controller/Map/metadataworker.h \
    Controller/Map/scanmapworker.h \
    Controller/Map/sendnewmapworker.h \
    Controller/Paths/pathscontroller.h \
    Controller/Points/pointscontroller.h \
    Controller/Robots/cmdrobotworker.h \
    Controller/Robots/commandcontroller.h \
    Controller/Robots/lasercontroller.h \
    Controller/Robots/robotpositionworker.h \
    Controller/Robots/robotscontroller.h \
    Controller/Robots/robotserverworker.h \
    Controller/Robots/teleopworker.h \
    Controller/Settings/settingscontroller.h \
    Controller/TopLayout/toplayoutcontroller.h \
    Controller/mainwindow.h \
    Helper/helper.h \
    Model/Map/map.h \
    Model/Other/graphicitemstate.h \
    Model/Other/position.h \
    Model/Other/xmlparser.h \
    Model/Paths/pathpoint.h \
    Model/Paths/paths.h \
    Model/Points/point.h \
    Model/Points/points.h \
    Model/Robots/robot.h \
    Model/Robots/robots.h \
    Model/Settings/settings.h \
    View/BottomLayout/bottomlayout.h \
    View/LeftMenu/leftmenu.h \
    View/LeftMenu/leftmenuwidget.h \
    View/LeftMenu/topleftmenu.h \
    View/Map/customqgraphicsview.h \
    View/Map/drawobstacles.h \
    View/Map/editmapview.h \
    View/Map/editmapwidget.h \
    View/Map/mapleftwidget.h \
    View/Map/mapview.h \
    View/Map/mergemapgraphicsitem.h \
    View/Map/mergemaplistitemwidget.h \
    View/Map/mergemaplistwidget.h \
    View/Map/mergemapwidget.h \
    View/Map/recoverpositionmapgraphicsitem.h \
    View/Map/robotpositionrecovery.h \
    View/Map/scanmapgraphicsitem.h \
    View/Map/scanmaplistitemwidget.h \
    View/Map/scanmapwidget.h \
    View/Map/teleopwidget.h \
    View/Other/customlabel.h \
    View/Other/customlineedit.h \
    View/Other/custompushbutton.h \
    View/Other/customrobotdialog.h \
    View/Other/customscrollarea.h \
    View/Other/spacewidget.h \
    View/Other/stylesettings.h \
    View/Paths/displaypathgroup.h \
    View/Paths/displayselectedpath.h \
    View/Paths/groupspathsbuttongroup.h \
    View/Paths/groupspathswidget.h \
    View/Paths/pathbuttongroup.h \
    View/Paths/pathcreationwidget.h \
    View/Paths/pathpainter.h \
    View/Paths/pathpointcreationwidget.h \
    View/Paths/pathpointlist.h \
    View/Paths/pathwidget.h \
    View/Points/createpointwidget.h \
    View/Points/displayselectedgroup.h \
    View/Points/displayselectedpoint.h \
    View/Points/displayselectedpointrobots.h \
    View/Points/groupbuttongroup.h \
    View/Points/pointbuttongroup.h \
    View/Points/pointsleftwidget.h \
    View/Points/pointview.h \
    View/Robots/commandmessagebox.h \
    View/Robots/editselectedrobotwidget.h \
    View/Robots/robotbtngroup.h \
    View/Robots/robotsleftwidget.h \
    View/Robots/robotview.h \
    View/Settings/settingswidget.h \
    View/TopLayout/toplayoutwidget.h \
    View/Map/robotpositionrecoverylistitemwidget.h


FORMS += Controller/mainwindow.ui

SOURCES += main.cpp \
    Controller/Map/localmapworker.cpp \
    Controller/Map/mapcontroller.cpp \
    Controller/Map/metadataworker.cpp \
    Controller/Map/scanmapworker.cpp \
    Controller/Map/sendnewmapworker.cpp \
    Controller/Paths/pathscontroller.cpp \
    Controller/Points/pointscontroller.cpp \
    Controller/Robots/cmdrobotworker.cpp \
    Controller/Robots/commandcontroller.cpp \
    Controller/Robots/lasercontroller.cpp \
    Controller/Robots/robotpositionworker.cpp \
    Controller/Robots/robotscontroller.cpp \
    Controller/Robots/robotserverworker.cpp \
    Controller/Robots/teleopworker.cpp \
    Controller/Settings/settingscontroller.cpp \
    Controller/TopLayout/toplayoutcontroller.cpp \
    Controller/mainwindow.cpp \
    Helper/helper.cpp \
    Model/Map/map.cpp \
    Model/Other/position.cpp \
    Model/Other/xmlparser.cpp \
    Model/Paths/pathpoint.cpp \
    Model/Paths/paths.cpp \
    Model/Points/point.cpp \
    Model/Points/points.cpp \
    Model/Robots/robot.cpp \
    Model/Robots/robots.cpp \
    Model/Settings/settings.cpp \
    View/BottomLayout/bottomlayout.cpp \
    View/LeftMenu/leftmenu.cpp \
    View/LeftMenu/leftmenuwidget.cpp \
    View/LeftMenu/topleftmenu.cpp \
    View/Map/customqgraphicsview.cpp \
    View/Map/drawobstacles.cpp \
    View/Map/editmapview.cpp \
    View/Map/editmapwidget.cpp \
    View/Map/mapleftwidget.cpp \
    View/Map/mapview.cpp \
    View/Map/mergemapgraphicsitem.cpp \
    View/Map/mergemaplistitemwidget.cpp \
    View/Map/mergemaplistwidget.cpp \
    View/Map/mergemapwidget.cpp \
    View/Map/recoverpositionmapgraphicsitem.cpp \
    View/Map/robotpositionrecovery.cpp \
    View/Map/scanmapgraphicsitem.cpp \
    View/Map/scanmaplistitemwidget.cpp \
    View/Map/scanmapwidget.cpp \
    View/Map/teleopwidget.cpp \
    View/Other/customlabel.cpp \
    View/Other/customlineedit.cpp \
    View/Other/custompushbutton.cpp \
    View/Other/customrobotdialog.cpp \
    View/Other/customscrollarea.cpp \
    View/Other/spacewidget.cpp \
    View/Paths/displaypathgroup.cpp \
    View/Paths/displayselectedpath.cpp \
    View/Paths/groupspathsbuttongroup.cpp \
    View/Paths/groupspathswidget.cpp \
    View/Paths/pathbuttongroup.cpp \
    View/Paths/pathcreationwidget.cpp \
    View/Paths/pathpainter.cpp \
    View/Paths/pathpointcreationwidget.cpp \
    View/Paths/pathpointlist.cpp \
    View/Paths/pathwidget.cpp \
    View/Points/createpointwidget.cpp \
    View/Points/displayselectedgroup.cpp \
    View/Points/displayselectedpoint.cpp \
    View/Points/displayselectedpointrobots.cpp \
    View/Points/groupbuttongroup.cpp \
    View/Points/pointbuttongroup.cpp \
    View/Points/pointsleftwidget.cpp \
    View/Points/pointview.cpp \
    View/Robots/commandmessagebox.cpp \
    View/Robots/editselectedrobotwidget.cpp \
    View/Robots/robotbtngroup.cpp \
    View/Robots/robotsleftwidget.cpp \
    View/Robots/robotview.cpp \
    View/Settings/settingswidget.cpp \
    View/TopLayout/toplayoutwidget.cpp \
    View/Map/robotpositionrecoverylistitemwidget.cpp

RESOURCES += Resources/resources.qrc

DISTFILES += README \
    points.xml \
    Resources/map_ori.png \
    Resources/robot_ori.png \
    Resources/gtrobot-1.pgm \
    Resources/gtrobot-1.pgm.pgm


