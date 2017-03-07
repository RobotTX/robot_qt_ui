import QtQuick 2.7
import QtQuick.Controls 2.0
import "../Robot"
import "../Path"
import "../Point"
import "../Map"
import "../Settings"
import "../../Helper/style.js" as Style
import "../../Model/Point"

Frame {
    id: mainMenuViewsFrame
    objectName: "mainMenuViewsFrame"
    visible: !(currentMenu == -1)
    width: Style.menuWidth
    padding: 0
    property Point pointModel
    property int currentMenu: -1

    RobotMenu {
        id: robotMenu
        visible: index == currentMenu
    }

    PathMenu {
        id: pathMenu
        visible: index == currentMenu
    }

    PointMenu {
        id: pointMenu
        visible: index == currentMenu
        _pointModel: pointModel
    }

    MapMenu {
        id: mapMenu
        visible: index == currentMenu
    }

    SettingsMenu {
        id: settingsMenu
        visible: index == currentMenu
    }

    /// We show the given menu in txt
    function showMenu(txt) {
        switch(txt){
            case 'Robot':
                if(robotMenu.visible)
                    mainMenuViewsFrame.currentMenu = -1;
                else
                    mainMenuViewsFrame.currentMenu = 0;
            break;
            case 'Path':
                if(pathMenu.visible)
                    mainMenuViewsFrame.currentMenu = -1;
                else
                    mainMenuViewsFrame.currentMenu = 1;
            break;
            case 'Point':
                if(pointMenu.visible)
                    mainMenuViewsFrame.currentMenu = -1;
                else
                    mainMenuViewsFrame.currentMenu = 2;
            break;
            case 'Map':
                if(mapMenu.visible)
                    mainMenuViewsFrame.currentMenu = -1;
                else
                    mainMenuViewsFrame.currentMenu = 3;
            break;
            case 'Settings':
                if(settingsMenu.visible)
                    mainMenuViewsFrame.currentMenu = -1;
                else
                    mainMenuViewsFrame.currentMenu = 4;
            break;
        }
    }
}
