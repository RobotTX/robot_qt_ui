import QtQuick 2.7
import QtQuick.Controls 1.4
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.0
import QtQuick.Window 2.0
import "Model/Point"
import "Model/Path"
import "Model/Robot"
import "Model/Speech"
import "Model/Tutorial/"
import "View/Custom"
import "View/MainMenu"
import "View/MapView"
import "View/Map"
import "View/EditMap"
import "View/Point"
import "View/Speech"
import "View/Robot"
import "View/ScanMap"
import "View/Tutorial/"
import "View/Settings"

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

    Component.onCompleted: {
        setX(Screen.width / 2 - width / 2);
        setY(Screen.height / 2 - height / 2);
    }

    // To save the current configuration -> zoom, center (paths and points retrieved on the c++ side)
    signal mapConfig(string file_name, double zoom, double centerX, double centerY, int mapRotation)
    signal shortcutAddRobot()
    signal shortcutDeleteRobot()
    signal requestOrSendMap(string ip, bool request)
    signal setMessageTop(int status, string msg)
    signal test()
    signal disco(string ip)

    onSetMessageTop: mapView.setMessageTop(status, msg)

    property bool useTmpPathModel: false
    property bool useRobotPathModel: false
    property real batteryWarningThreshold: 20
    property string langue: _robotModel.langue

    title:langue == "English" ? qsTr("小Go去哪儿") : qsTr("Go Anywhere")

    onClosing: {
        scanMap.close();
        editMap.close();
//        Qt.quit();
    }

    Item {
        Points {
            id: _pointModel
            objectName: "pointModel"
            langue: applicationWindow.langue
            onSetMessageTop: mapView.setMessageTop(status, msg)
            onSaveCurrentHome: {
                mainFrame.currentMenu = 2
            }
            onEditPointB: {
                mainFrame.currentMenu = 2
            }
        }

        Speechs {
            id: _speechModel
            objectName: "speechModel"
            langue: applicationWindow.langue
            onSetMessageTop: mapView.setMessageTop(status, msg)
        }

        Paths {
            id: _pathModel
            objectName: "pathModel"
            langue: applicationWindow.langue
            onSetMessageTop: mapView.setMessageTop(status, msg)
            onSaveCurrentPath: {
                mainFrame.currentMenu = 1
            }
        }

        Paths {
            id: _tmpPathModel
            objectName: "tmpPathModel"
            langue: applicationWindow.langue
            Component.onCompleted: clearTmpPath();
        }

        Robots {
            id: _robotModel
            objectName: "robotModel"
            onSetMessageTop: mapView.setMessageTop(status, msg)
            batteryWarningThreshold: applicationWindow.batteryWarningThreshold
            langue: applicationWindow.langue
        }

        Tutorial {
            id: _tutorial
            objectName: "tutorialModel"
            langue: applicationWindow.langue
        }

        /// NOTE Just for testing, to remove later
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
        Shortcut {
            sequence: ";"
            onActivated: test()
        }

        EditMap {
            id: editMap
            imgSource: mapView.mapSrc
            tutorial: _tutorial
            langue: applicationWindow.langue
        }

        ScanMap {
            id: scanMap
            robotModel: _robotModel
            tutorial: _tutorial
            langue: applicationWindow.langue
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
            langue: applicationWindow.langue
            currentMenu: mainFrame.currentMenu
            onSelectMenu: mainFrame.currentMenu = index
//            visible: mainFrame.currentMenu === 5 ? false : true
            visible: true
        }

        MainMenuViews {
            id: mainMenuViews
            z: 1
            pointModel: _pointModel
            speechModel: _speechModel
            tmpPointView: mapView.tmpPointView
            pathModel: _pathModel
            tmpPathModel: _tmpPathModel
            robotModel: _robotModel
            tutorial: _tutorial
            currentMenu: mainFrame.currentMenu
            batteryWarningThreshold: applicationWindow.batteryWarningThreshold
            langue: applicationWindow.langue
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
            langue: applicationWindow.langue
            useTmpPathModel: applicationWindow.useTmpPathModel
            useRobotPathModel: applicationWindow.useRobotPathModel
            anchors {
                left: mainMenuViews.visible ? mainMenuViews.right : mainMenu.right
                top: parent.top
                right: parent.right
                bottom: parent.bottom
            }
            onDoubleClickedOnMap: {
                console.log("size map " + width + " " + height);
                mainFrame.currentMenu = 2;
                mainMenuViews.doubleClickedOnMap(mouseX, mouseY);
            }
        }
    }

    CustomDialog {
        id: dialog
        x: applicationWindow.width / 2 - width / 2
        y: applicationWindow.height / 2 - height / 2

        onAccepted: requestOrSendMap(ip, false)
        onRejected: requestOrSendMap(ip, true)
    }

    CustomDialog {
        id: warningDialog
        x: applicationWindow.width / 2 - width / 2
        y: applicationWindow.height / 2 - height / 2
    }

//    TextField {
//        id: oldmap
//        text: " old map"
//        color: "red"
//        font.bold: true
//        font.pointSize: 13
//    }

//    TextField {
//        id: newmap
//        text: " new map"
//        color: "green"
//        font.bold: true
//        font.pointSize: 13
//    }

    Text {
        id: oldmap
        text: "old map"
        color: "red"
        font.bold: true
        font.pointSize: 13
        visible: false
    }

    Text {
        id: newmap
        text: "new map"
        color: "green"
        font.bold: true
        font.pointSize: 13
        visible: false
    }


    function openMapChoiceMessageDialog(ip, robotIsOlder){
        var title_ = ''
        var message1 = ''
        var message2 = ''
        var rejectMessage_ = ''
        var acceptMessage_ = ''
        if(dialog.visible){
            /// TODO fix this (if more than 1 robot connect, has a wrong map, and we were already asking to choose a map for the previous robot), MORE ROBOTS NEEDED
            console.log("We are already choosing a map for the robot, try again later");
        } else {
            if (langue == "English") {
                title_ = "选择使用哪一个地图"
                message1 = "机器人 " + _robotModel.getName(ip) +  " 有一个旧地图"
                message2 = "机器人 " + _robotModel.getName(ip) + " 有一个新地图"
                rejectMessage_ = "机器人"
                acceptMessage_ = "应用"
            } else {
                title_ = "Choose which map to use"
                message1 = "The robot " +  _robotModel.getName(ip) + " has an old map"
                message2 = "The robot " +  _robotModel.getName(ip) + " has a new map.\n\n\tWhich map do you want to use ?"
                rejectMessage_ = "Robot"
                acceptMessage_ = "Application"
            }


            dialog.title = title_;
            dialog.ip = ip;
            dialog.message = robotIsOlder ? message1 : message2;
            dialog.rejectMessage = rejectMessage_;
            dialog.acceptMessage = acceptMessage_;
            dialog.open();
        }
    }

    function openWarningDialog(title, msg){
        warningDialog.title = title;
        warningDialog.message = msg;
        warningDialog.acceptMessage = "Ok";
        warningDialog.open();
    }

    function emitMapConfig(file_name){
        var message = ''
        if (langue == "English") {
            message = "保存地图到 \"" + file_name + "\""
        } else {
            message = "Saved the map to \"" + file_name + "\""
        }

        console.log(mapView.pointModel.count + " " + mapView.scale + " " + mapView.centerX + " " + mapView.centerY);
        console.log("map config");
        applicationWindow.mapConfig(file_name, mapView.zoom, mapView.centerX, mapView.centerY, mapView.getMapRotation());
        mapView.setMessageTop(3, message);
    }

    function setBatteryThreshold(threshold){
        batteryWarningThreshold = threshold;
    }

    function openScanWindowForAutomaticScan(ip){
        scanMap.show();
        scanMap._scanMapLeftMenu.scanningRobots.addRobot("Automatic scan", ip, true)
    }

}

