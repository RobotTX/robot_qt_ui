import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Robot"
import "../Path"
import "../Point"
import "../Speech"
import "../Map"
import "../Guide"
import "../../Model/"
import "../Settings"
import "../../Helper/style.js" as Style
import "../../Model/Point"
import "../../Model/Speech"
import "../../Model/Path"
import "../../Model/Robot"
import "../../Model/Tutorial/"

/**
  * The left panel displaying the opened menu
  */
Frame {
    id: mainMenuViewsFrame
    visible: !(currentMenu == -1)
    width: currentMenu === 5 ? parent.width : Style.menuWidth
//    width: Style.menuWidth
    padding: 0
    property Points pointModel
    property Speechs speechModel
    property PointView tmpPointView
    property Paths pathModel
    property Paths tmpPathModel
    property Robots robotModel
    property Tutorial tutorial
    property real batteryWarningThreshold
    property string langue
    property string inputNameWifi
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
        onSetMessageTop: mainMenuViewsFrame.setMessageTop(status, msg)
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
        speechModel: mainMenuViewsFrame.speechModel
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
        pathModel: mainMenuViewsFrame.pathModel
        tmpPointView: mainMenuViewsFrame.tmpPointView
        langue: mainMenuViewsFrame.langue
        onCloseMenu: mainMenuViewsFrame.closeMenu()
        onSetMessageTop: mainMenuViewsFrame.setMessageTop(status, msg)
    }

    SpeechMenu {
        id: speechMenu
        visible: currentMenu == 3
        speechModel: mainMenuViewsFrame.speechModel
        robotModel: mainMenuViewsFrame.robotModel
        langue: mainMenuViewsFrame.langue
        onCloseMenu: mainMenuViewsFrame.closeMenu()
        onSetMessageTop: mainMenuViewsFrame.setMessageTop(status, msg)
    }
    MapMenu {
        id: mapMenu
        visible: currentMenu == 4
        langue: mainMenuViewsFrame.langue
        onCloseMenu: mainMenuViewsFrame.closeMenu()
        onSavePosition: mainMenuViewsFrame.savePosition()
        onSaveMap: mainMenuViewsFrame.saveMap(file_name)
    }

    GuideMenu {
        id: guideMenu
        visible: currentMenu == 5
        pointModel: mainMenuViewsFrame.pointModel
        robotModel: mainMenuViewsFrame.robotModel
        pathModel: mainMenuViewsFrame.pathModel
        langue: mainMenuViewsFrame.langue
        tmpPathModel: mainMenuViewsFrame.tmpPathModel
        speechModel: mainMenuViewsFrame.speechModel
        currentMenu: mainMenuViewsFrame.currentMenu
        onUseTmpPathModel: mainMenuViewsFrame.useTmpPathModel(use)
        onUseRobotPathModel: mainMenuViewsFrame.useRobotPathModel(use)
        onCloseMenu: mainMenuViewsFrame.closeMenu()
        onSetMessageTop: mainMenuViewsFrame.setMessageTop(status, msg)
    }

    SettingsMenu {
        id: settingsMenu
        tutorial: mainMenuViewsFrame.tutorial
        robotModel: mainMenuViewsFrame.robotModel
        batteryWarningThreshold: mainMenuViewsFrame.batteryWarningThreshold
        pathModel: mainMenuViewsFrame.pathModel
        langue: mainMenuViewsFrame.langue
        visible: currentMenu == 6
        onCloseMenu: mainMenuViewsFrame.closeMenu()
        inputNameWifi: mainMenuViewsFrame.inputNameWifi
    }

    function doubleClickedOnMap(mouseX, mouseY){
        pointMenu.doubleClickedOnMap(mouseX, mouseY);
    }
}
