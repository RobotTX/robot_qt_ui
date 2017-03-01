import QtQuick 2.7
import QtQuick.Controls 2.0
import "../Robot"
import "../Path"
import "../Point"
import "../Map"
import "../Settings"

Frame {
    id: mainMenuViewsFrame
    objectName: "mainMenuViewsFrame"
    visible: false
    width: 270
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
        console.log("showMenu Got message:", txt);

        switch(txt){
            case 'Robot':
                console.log("Display Robot page");
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
                console.log("Display Path page");
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
                console.log("Display Point page");
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
                console.log("Display Map page");
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
                console.log("Display Settings page");
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
