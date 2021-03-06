import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

/**
  * List of buttons of the left menu
  */
Frame {
    id: mainMenuFrame
    /// The index of the current menu
    property int currentMenu
    property string langue

    width: currentMenu === 5 ? 0 : Style.mainMenuWidth
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
        txt: langue == "English" ? "ROBOT" :  "机器人"
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
        txt: langue == "English" ? "PATH" : "路径"
        imgSrc: "qrc:/icons/path"
        anchors.top: robotButton.bottom
        onClicked: checked ? selectMenu(1) : selectMenu(-1)
        checked: currentMenu == 1
    }

    MenuButton {
        id: pointButton
        txt: langue == "English" ? "POINT" : "目标点"
        imgSrc: "qrc:/icons/point"
        anchors.top: pathButton.bottom
        onClicked: checked ? selectMenu(2) : selectMenu(-1)
        checked: currentMenu == 2
    }

    MenuButton {
        id: speechButton
        txt: langue == "English" ? "SPEECH" : "语音"
        imgSrc: "qrc:/icons/speech"
        anchors.top: pointButton.bottom
        onClicked: checked ? selectMenu(3) : selectMenu(-1)
        checked: currentMenu == 3
    }

    MenuButton {
        id: mapButton
        txt: langue == "English" ? "MAP" : "地图"
        imgSrc: "qrc:/icons/map"
        anchors.top: speechButton.bottom
        onClicked: checked ? selectMenu(4) : selectMenu(-1)
        checked: currentMenu == 4
    }

//    MenuButton {
//        id: guideButton
//        txt: langue == "English" ? "地图" : "Guide"
//        imgSrc: "qrc:/icons/robot"
//        anchors.top: mapButton.bottom
//        onClicked: checked ? selectMenu(5) : selectMenu(-1)
//        checked: currentMenu == 5
//    }

    MenuButton {
        id: settingsButton
        txt: langue == "English" ? "SETTING" : "设置"
        imgSrc: "qrc:/icons/settings"
        anchors.bottom: parent.bottom
        onClicked: checked ? selectMenu(6) : selectMenu(-1)
        checked: currentMenu == 6
    }
}
