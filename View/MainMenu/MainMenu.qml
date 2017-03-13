import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

/**
  * List of buttons of the left menu
  */
Frame {
    id: mainMenuFrame
    objectName: "mainMenuFrame"
    /// The index of the current menu
    property int currentMenu

    width: Style.mainMenuWidth
    padding: 0

    anchors {
        left: parent.left
        top: parent.top
        bottom: parent.bottom
    }

    background: Rectangle {
        color: Style.darkGrey
    }

    signal selectMenu(int index)

    MenuButton {
        id: robotButton
        txt: "Robot"
        imgSrc: "qrc:/icons/robot"
        anchors {
            left: parent.left
            top: parent.top
        }
        onClicked: checked ? selectMenu(0) : selectMenu(-1)
        checked: currentMenu == 0
    }

    MenuButton {
        id: pathButton
        txt: "Path"
        imgSrc: "qrc:/icons/path"
        anchors.top: robotButton.bottom
        onClicked: checked ? selectMenu(1) : selectMenu(-1)
        checked: currentMenu == 1
    }

    MenuButton {
        id: pointButton
        txt: "Point"
        imgSrc: "qrc:/icons/point"
        anchors.top: pathButton.bottom
        onClicked: checked ? selectMenu(2) : selectMenu(-1)
        checked: currentMenu == 2
    }

    MenuButton {
        id: mapButton
        txt: "Map"
        imgSrc: "qrc:/icons/map"
        anchors.top: pointButton.bottom
        onClicked: checked ? selectMenu(3) : selectMenu(-1)
        checked: currentMenu == 3
    }

    MenuButton {
        id: settingsButton
        txt: "Settings"
        imgSrc: "qrc:/icons/settings"
        anchors.bottom: parent.bottom
        onClicked: checked ? selectMenu(4) : selectMenu(-1)
        checked: currentMenu == 4
    }
}
