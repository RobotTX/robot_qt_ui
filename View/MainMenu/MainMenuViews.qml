import QtQuick 2.7
import QtQuick.Controls 2.0
import "../Robot"
import "../Path"
import "../Point"
import "../Map"
import "../Settings"
import "../../Helper/style.js" as Style

Frame {
    id: mainMenuViewsFrame
    objectName: "mainMenuViewsFrame"
    visible: false
    width: Style.menuWidth
    padding: 0

    RobotMenu {
        id: robotMenu
        visible: false
    }

    PathMenu {
        id: pathMenu
        visible: false
    }

    PointMenu {
        id: pointMenu
        visible: false
    }

    MapMenu {
        id: mapMenu
        visible: false
    }

    SettingsMenu {
        id: settingsMenu
        visible: false
    }

    /// We show the given menu in txt
    function showMenu(txt) {
        switch(txt){
            case 'Robot':
                if(robotMenu.visible){
                    robotMenu.visible = false;
                    mainMenuViewsFrame.visible = false;
                } else {
                    robotMenu.visible = true;
                    mainMenuViewsFrame.visible = true;

                    pathMenu.visible = false;
                    pointMenu.visible = false;
                    mapMenu.visible = false;
                    settingsMenu.visible = false;
                }
            break;
            case 'Path':
                if(pathMenu.visible){
                    pathMenu.visible = false;
                    mainMenuViewsFrame.visible = false;
                } else {
                    pathMenu.visible = true;
                    mainMenuViewsFrame.visible = true;

                    robotMenu.visible = false;
                    pointMenu.visible = false;
                    mapMenu.visible = false;
                    settingsMenu.visible = false;
                }
            break;
            case 'Point':
                if(pointMenu.visible){
                    pointMenu.visible = false;
                    mainMenuViewsFrame.visible = false;
                } else {
                    pointMenu.visible = true;
                    mainMenuViewsFrame.visible = true;

                    robotMenu.visible = false;
                    pathMenu.visible = false;
                    mapMenu.visible = false;
                    settingsMenu.visible = false;
                }
            break;
            case 'Map':
                if(mapMenu.visible){
                    mapMenu.visible = false;
                    mainMenuViewsFrame.visible = false;
                } else {
                    mapMenu.visible = true;
                    mainMenuViewsFrame.visible = true;

                    robotMenu.visible = false;
                    pathMenu.visible = false;
                    pointMenu.visible = false;
                    settingsMenu.visible = false;
                }
            break;
            case 'Settings':
                if(settingsMenu.visible){
                    settingsMenu.visible = false;
                    mainMenuViewsFrame.visible = false;
                } else {
                    settingsMenu.visible = true;
                    mainMenuViewsFrame.visible = true;

                    robotMenu.visible = false;
                    pathMenu.visible = false;
                    pointMenu.visible = false;
                    mapMenu.visible = false;
                }
            break;
        }
    }
}
