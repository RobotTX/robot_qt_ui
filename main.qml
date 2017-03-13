import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.0
import "View/MainMenu"
import "View/MapView"
import "Model/Point"
import "View/Point"

ApplicationWindow {
    id: applicationWindow
    objectName: "applicationWindow"
    visible: true
    width: 1000
    height: 700
    minimumWidth: 800
    minimumHeight: 600
    title: qsTr("Gobot :)")

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
    }
}
