import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.0
import "View/MainMenu"
import "View/MapView"
import "Model/Point"

ApplicationWindow {
    id: applicationWindow
    objectName: "applicationWindow"
    visible: true
    width: 1000
    height: 700
    minimumWidth: 800
    minimumHeight: 600
    title: qsTr("Gobot :)")

    Keys.onPressed: {
        console.log("space");
    }

    Item {
        Points {
            id: _pointModel
            objectName: "pointModel"
        }
    }

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
            pointModel: _pointModel
            anchors {
                left: mainMenu.right
                top: parent.top
                bottom: parent.bottom
            }
        }

        MapView {
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
