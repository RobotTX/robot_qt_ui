import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.0
import QtQuick.Window 2.0
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

    Item {
        Points {
            id: _pointModel
            objectName: "pointModel"
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
            currentMenu: layout.currentMenu
            anchors {
                left: mainMenu.right
                top: parent.top
                bottom: parent.bottom
            }
            onCloseMenu: layout.currentMenu = -1
            onSavePosition: mapView.emitPosition()
            onSaveMap: applicationWindow.emitMapConfig(file_name)
        }

        MapView {
            id: mapView
            pointModel: _pointModel
            anchors {
                left: (mainMenuViews.visible) ? mainMenuViews.right : mainMenu.right
                top: parent.top
                right: parent.right
                bottom: parent.bottom
            }
        }


        EditMap {
            id: editMap

        }
    }

    function emitMapConfig(file_name){
        console.log(mapView.pointModel.count + " " + mapView.scale + " " + mapView.centerX + " " + mapView.centerY);
        console.log("map config");
        mapView._topView.label.text = "The current configuration of the map has been saved";
        applicationWindow.mapConfig(file_name, mapView.zoom, mapView.centerX, mapView.centerY);
    }
}
