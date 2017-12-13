import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Robot"
import "../Path"
import "../Point"
import "../Map"
import "../../Model/"
import "../Settings"
import "../../Helper/style.js" as Style
import "../../Model/Point"
import "../../Model/Path"
import "../../Model/Robot"
import "../../Model/Tutorial/"

/**
  * The left panel displaying the opened menu
  */
Frame {
    id: mainMenuViewsFrame
    visible: !(currentMenu == -1)
    width: Style.menuWidth
    padding: 0
    property Points pointModel
    property PointView tmpPointView
    property Paths pathModel
    property Paths tmpPathModel
    property Robots robotModel
    property Tutorial tutorial
    property real batteryWarningThreshold
    property string langue
    signal useTmpPathModel(bool use)
    signal useRobotPathModel(bool use)
    signal setMessageTop(int status, string msg)

    /// The index of the current menu
    property int currentMenu

    signal closeMenu()
    signal savePosition()

    signal saveMap(string file_name)

    RobotMenu {
        id: robotMenu
        visible: currentMenu == 0
        robotModel: mainMenuViewsFrame.robotModel
        pathModel: mainMenuViewsFrame.pathModel
        pointModel: mainMenuViewsFrame.pointModel
        batteryWarningThreshold: mainMenuViewsFrame.batteryWarningThreshold
        langue: mainMenuViewsFrame.langue
        onUseRobotPathModel: mainMenuViewsFrame.useRobotPathModel(use)
        onCloseMenu: mainMenuViewsFrame.closeMenu()
    }

    PathMenu {
        id: pathMenu
        visible: currentMenu == 1
        robotModel: mainMenuViewsFrame.robotModel
        tmpPathModel: mainMenuViewsFrame.tmpPathModel
        pathModel: mainMenuViewsFrame.pathModel
        pointModel: mainMenuViewsFrame.pointModel
        langue: mainMenuViewsFrame.langue
        onUseTmpPathModel: mainMenuViewsFrame.useTmpPathModel(use)
        onUseRobotPathModel: mainMenuViewsFrame.useRobotPathModel(use)
        onCloseMenu: mainMenuViewsFrame.closeMenu()
        onSetMessageTop: mainMenuViewsFrame.setMessageTop(status, msg)
    }

    PointMenu {
        id: pointMenu
        visible: currentMenu == 2
        pointModel: mainMenuViewsFrame.pointModel
        robotModel: mainMenuViewsFrame.robotModel
        tmpPointView: mainMenuViewsFrame.tmpPointView
        langue: mainMenuViewsFrame.langue
        onCloseMenu: mainMenuViewsFrame.closeMenu()
        onSetMessageTop: mainMenuViewsFrame.setMessageTop(status, msg)
    }

    MapMenu {
        id: mapMenu
        visible: currentMenu == 3
        langue: mainMenuViewsFrame.langue
        onCloseMenu: mainMenuViewsFrame.closeMenu()
        onSavePosition: mainMenuViewsFrame.savePosition()
        onSaveMap: mainMenuViewsFrame.saveMap(file_name)
    }

    SettingsMenu {
        id: settingsMenu
        tutorial: mainMenuViewsFrame.tutorial
        robotModel: mainMenuViewsFrame.robotModel
        batteryWarningThreshold: mainMenuViewsFrame.batteryWarningThreshold
        langue: mainMenuViewsFrame.langue
        visible: currentMenu == 4
        onCloseMenu: mainMenuViewsFrame.closeMenu()
    }

    function doubleClickedOnMap(mouseX, mouseY){
        pointMenu.doubleClickedOnMap(mouseX, mouseY);
    }
}
