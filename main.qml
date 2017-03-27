import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.0
import QtQuick.Window 2.0
import "Model/Point"
import "Model/Path"
import "Model/Robot"
import "View/MainMenu"
import "View/MapView"
import "View/Map"
import "Model/Point"
import "View/Point"

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
    title: qsTr("Gobot :)")
    Component.onCompleted: {
            setX(Screen.width / 2 - width / 2);
            setY(Screen.height / 2 - height / 2);
        }

    // TODO add paths
    // To save the current configuration -> zoom, center (paths and points retrieved on the c++ side)
    signal mapConfig(string file_name, double zoom, double centerX, double centerY)
    signal shortcutAddRobot()
    signal shortcutDeleteRobot()

    property bool useTmpPathModel: false

    Item {
        Points {
            id: _pointModel
            objectName: "pointModel"
        }

        Paths {
            id: _pathModel
            objectName: "pathModel"
        }

        Paths {
            id: _tmpPathModel
            objectName: "tmpPathModel"
            Component.onCompleted: clearTmpPath();
        }

        Robots {
            id: _robotModel
            objectName: "robotModel"
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
    }

    Frame {
        id: layout
        spacing: 0
        padding: 0
        anchors.fill: parent
        /// The index of the current menu
        property int currentMenu: -1

        MainMenu {
            id: mainMenu
            z: 1
            currentMenu: layout.currentMenu
            onSelectMenu: layout.currentMenu = index
        }

        MainMenuViews {
            id: mainMenuViews
            z: 1
            pointModel: _pointModel
            tmpPointView: mapView.tmpPointView
            pathModel: _pathModel
            tmpPathModel: _tmpPathModel
            robotModel: _robotModel
            currentMenu: layout.currentMenu
            anchors {
                left: mainMenu.right
                top: parent.top
                bottom: parent.bottom
            }
            onCloseMenu: layout.currentMenu = -1
            onSavePosition: mapView.emitPosition()
            onSaveMap: applicationWindow.emitMapConfig(file_name)
            onUseTmpPathModel: applicationWindow.useTmpPathModel = use
        }

        MapView {
            id: mapView
            pointModel: _pointModel
            pathModel: _pathModel
            tmpPathModel: _tmpPathModel
            robotModel: _robotModel
            useTmpPathModel: applicationWindow.useTmpPathModel
            anchors {
                left: mainMenuViews.visible ? mainMenuViews.right : mainMenu.right
                top: parent.top
                right: parent.right
                bottom: parent.bottom
            }

        }

        EditMap {
            id: editMap
            onVisibleChanged: {
                console.log("just changed to " + mapView.mapSrc);
                imgSource = mapView.mapSrc;
            }
        }
    }

    function emitMapConfig(file_name){
        console.log(mapView.pointModel.count + " " + mapView.scale + " " + mapView.centerX + " " + mapView.centerY);
        console.log("map config");
        mapView.topView.label.text = "The current configuration of the map has been saved";
        applicationWindow.mapConfig(file_name, mapView.zoom, mapView.centerX, mapView.centerY);
    }

    function reloadMapImage(file_name){
        // little trick as the binding property does not allow the map to be reloaded unless the filename changes
        console.log("updating");
        mapView.mapSrc = "qrc:/icons/hand";
        mapView.mapSrc = file_name;
    }
}
