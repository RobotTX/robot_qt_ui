import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0
import "View/MainMenu"
import "View/MapView"

ApplicationWindow {
    id: applicationWindow
    objectName: "applicationWindow"
    visible: true
    width: 1000
    height: 700
    minimumWidth: 800
    minimumHeight: 600
    title: qsTr("Gobot :)")

    ColumnLayout {
        spacing: 0
        anchors.fill: parent

        MainMenu {
            id: mainMenu
            z: 1
        }

        MainMenuViews {
            id: mainMenuViews
            z: 1
            anchors {
                left: mainMenu.right
                top: parent.top
                bottom: parent.bottom
            }
        }

        MapView {
            anchors {
                left: (mainMenuViews.visible) ? mainMenuViews.right : mainMenu.right
                top: parent.top
                right: parent.right
                bottom: parent.bottom
            }
        }
    }
}
