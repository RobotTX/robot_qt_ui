import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Robot"
import "../Path"
import "../Point"
import "../Map"
import "../Settings"
import "../../Helper/style.js" as Style
import "../../Model/Point"
import "../../Model/Path"
import "../../Model/Robot"

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
    property real batteryWarningThreshold
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
        onUseRobotPathModel: mainMenuViewsFrame.useRobotPathModel(use)
        onCloseMenu: mainMenuViewsFrame.closeMenu()
    }

    PathMenu {
        id: pathMenu
        visible: currentMenu == 1
        tmpPathModel: mainMenuViewsFrame.tmpPathModel
        pathModel: mainMenuViewsFrame.pathModel
        pointModel: mainMenuViewsFrame.pointModel
        onUseTmpPathModel: mainMenuViewsFrame.useTmpPathModel(use)
        onUseRobotPathModel: mainMenuViewsFrame.useRobotPathModel(use)
        onCloseMenu: mainMenuViewsFrame.closeMenu()
        onSetMessageTop: mainMenuViewsFrame.setMessageTop(status, msg)
    }

    PointMenu {
        id: pointMenu
        visible: currentMenu == 2
        pointModel: mainMenuViewsFrame.pointModel
        tmpPointView: mainMenuViewsFrame.tmpPointView
        onCloseMenu: mainMenuViewsFrame.closeMenu()
        onSetMessageTop: mainMenuViewsFrame.setMessageTop(status, msg)
    }

    MapMenu {
        id: mapMenu
        visible: currentMenu == 3
        onCloseMenu: mainMenuViewsFrame.closeMenu()
        onSavePosition: mainMenuViewsFrame.savePosition()
        onSaveMap: mainMenuViewsFrame.saveMap(file_name)
    }

    SettingsMenu {
        id: settingsMenu
        robotModel: mainMenuViewsFrame.robotModel
        batteryWarningThreshold: mainMenuViewsFrame.batteryWarningThreshold
        visible: currentMenu == 4
        onCloseMenu: mainMenuViewsFrame.closeMenu()
    }

    function doubleClickedOnMap(mouseX, mouseY){
        pointMenu.doubleClickedOnMap(mouseX, mouseY);
    }
}
