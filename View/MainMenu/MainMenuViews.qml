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
    signal useTmpPathModel(bool use)

    /// The index of the current menu
    property int currentMenu

    signal closeMenu()
    signal savePosition()

    signal saveMap(string file_name)

    RobotMenu {
        id: robotMenu
        visible: currentMenu == 0
        onCloseMenu: mainMenuViewsFrame.closeMenu()
    }

    PathMenu {
        id: pathMenu
        visible: currentMenu == 1
        tmpPathModel: mainMenuViewsFrame.tmpPathModel
        pathModel: mainMenuViewsFrame.pathModel
        pointModel: mainMenuViewsFrame.pointModel
        onUseTmpPathModel: mainMenuViewsFrame.useTmpPathModel(use)
        onCloseMenu: mainMenuViewsFrame.closeMenu()
    }

    PointMenu {
        id: pointMenu
        visible: currentMenu == 2
        pointModel: mainMenuViewsFrame.pointModel
        tmpPointView: mainMenuViewsFrame.tmpPointView
        onCloseMenu: mainMenuViewsFrame.closeMenu()
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
        visible: currentMenu == 4
        onCloseMenu: mainMenuViewsFrame.closeMenu()
    }
}
