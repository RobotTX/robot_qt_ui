import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.0
import QtQuick.Window 2.0
import "Model/Point"
import "Model/Path"
import "Model/Robot"
import "Model/Point"
import "View/Custom"
import "View/MainMenu"
import "View/MapView"
import "View/Map"
import "View/Point"
import "View/Robot"
import "View/ScanMap"

ApplicationWindow {
    id: applicationWindow
    objectName: "applicationWindow"
    x: Screen.width / 2
    y: Screen.height / 2
    visible: true
    width: 1000
    height: 700
    minimumWidth: 800
    minimumHeight: 600
    title: qsTr("GT Gobot")
    Component.onCompleted: {
            setX(Screen.width / 2 - width / 2);
            setY(Screen.height / 2 - height / 2);
        }

    // TODO add paths
    // To save the current configuration -> zoom, center (paths and points retrieved on the c++ side)
    signal mapConfig(string file_name, double zoom, double centerX, double centerY)
    signal shortcutAddRobot()
    signal shortcutDeleteRobot()
    signal requestOrSendMap(string ip, bool request)
    signal setMessageTop(int status, string msg)

    onSetMessageTop: mapView.setMessageTop(status, msg)

    property bool useTmpPathModel: false
    property bool useRobotPathModel: false
    property real batteryWarningThreshold: 20

    Item {
        Points {
            id: _pointModel
            objectName: "pointModel"
            onSetMessageTop: mapView.setMessageTop(status, msg)
        }

        Paths {
            id: _pathModel
            objectName: "pathModel"
            onSetMessageTop: mapView.setMessageTop(status, msg)
        }

        Paths {
            id: _tmpPathModel
            objectName: "tmpPathModel"
            Component.onCompleted: clearTmpPath();
        }

        Robots {
            id: _robotModel
            objectName: "robotModel"
            onSetMessageTop: mapView.setMessageTop(status, msg)
        }

        /// TODO Just for testing, to remove later
        Shortcut {
            sequence: "."
            onActivated: shortcutAddRobot()
        }
        Shortcut {
            sequence: ","
            onActivated: shortcutDeleteRobot()
        }
        Shortcut {
            sequence: "/"
            onActivated: openMapChoiceMessageDialog("0", true)
        }
    }

    Frame {
        id: mainFrame
        spacing: 0
        padding: 0
        anchors.fill: parent
        /// The index of the current menu
        property int currentMenu: -1

        MainMenu {
            id: mainMenu
            z: 1
            currentMenu: mainFrame.currentMenu
            onSelectMenu: mainFrame.currentMenu = index
        }

        MainMenuViews {
            id: mainMenuViews
            z: 1
            pointModel: _pointModel
            tmpPointView: mapView.tmpPointView
            pathModel: _pathModel
            tmpPathModel: _tmpPathModel
            robotModel: _robotModel
            currentMenu: mainFrame.currentMenu
            batteryWarningThreshold: applicationWindow.batteryWarningThreshold
            anchors {
                left: mainMenu.right
                top: parent.top
                bottom: parent.bottom
            }
            onCloseMenu: mainFrame.currentMenu = -1
            onSavePosition: mapView.emitPosition()
            onSaveMap: applicationWindow.emitMapConfig(file_name)
            onUseTmpPathModel: applicationWindow.useTmpPathModel = use
            onUseRobotPathModel: applicationWindow.useRobotPathModel = use
            onSetMessageTop: mapView.setMessageTop(status, msg)
        }

        MapView {
            id: mapView
            pointModel: _pointModel
            pathModel: _pathModel
            tmpPathModel: _tmpPathModel
            robotModel: _robotModel
            useTmpPathModel: applicationWindow.useTmpPathModel
            useRobotPathModel: applicationWindow.useRobotPathModel
            anchors {
                left: mainMenuViews.visible ? mainMenuViews.right : mainMenu.right
                top: parent.top
                right: parent.right
                bottom: parent.bottom
            }
            onDoubleClickedOnMap: {
                mainFrame.currentMenu = 2;
                mainMenuViews.doubleClickedOnMap(mouseX, mouseY);
            }
        }

        EditMap {
            id: editMap
            onVisibleChanged: {
                console.log("just changed to " + mapView.mapSrc);
                imgSource = mapView.mapSrc;
            }
        }

        MergeMap {
            id: mergeMap
            robotModel: _robotModel
        }

        ScanMap {
            id: scanMap
            robotModel: _robotModel
        }
    }

    DualChoiceMessageDialog {
        id: dualChoiceMessageDialog
        x: parent.width / 2 - width / 2
        y: parent.height / 2 - height / 2

        onAccepted: requestOrSendMap(ip, false)
        onRejected: requestOrSendMap(ip, true)
    }

    function openMapChoiceMessageDialog(ip, robotIsOlder){
        if(dualChoiceMessageDialog.visible){
            /// TODO fix this (if more than 1 robot connect and has a wrong map, we were already asking to choose a map for the previous robot)
            console.log("We are already choosing a map for the robot, try again later");
        } else {
            dualChoiceMessageDialog.title = qsTr("Choose which map to use");
            dualChoiceMessageDialog.ip = ip;
            dualChoiceMessageDialog.message = (robotIsOlder ? "The robot " +  _robotModel.getName(ip) + " has a new map." : "The robot " +  _robotModel.getName(ip) + " has an old map.") + "\n\n\tWhich map do you want to use ?";
            dualChoiceMessageDialog.rejectMessage = "Robot";
            dualChoiceMessageDialog.acceptMessage = "Application";
            dualChoiceMessageDialog.open();
        }
    }

    function emitMapConfig(file_name){
        console.log(mapView.pointModel.count + " " + mapView.scale + " " + mapView.centerX + " " + mapView.centerY);
        console.log("map config");
        applicationWindow.mapConfig(file_name, mapView.zoom, mapView.centerX, mapView.centerY);
        mapView.setMessageTop(2, "Saved the map to \"" + file_name + "\"");
    }

    function reloadMapImage(file_name){
        // little trick as the binding property does not allow the map to be reloaded unless the filename changes
        console.log("updating");
        mapView.mapSrc = "qrc:/icons/hand";
        mapView.mapSrc = file_name;
    }

    function setBatteryThreshold(threshold){
        batteryWarningThreshold = threshold;
    }
}
