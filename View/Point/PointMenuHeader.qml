import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../Custom"

Frame {
    id: menuHeader
    padding: 0
    z: 2
    readonly property string txt: "Point"
    property string langue
    signal closeMenu(string txt)
    signal openCreatePointMenu()
    signal openCreateGroupMenu()

    height: Style.menuHeaderHeight
    anchors {
        left: parent.left
        top: parent.top
        right: parent.right
    }

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }


    SmallButton {
        id: closeBtn
        onClicked: menuHeader.closeMenu(txt)
        imgSrc: "qrc:/icons/closeBtn"
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: parent.left
        anchors.leftMargin: 11
    }

    Label {
        color: Style.midGrey2
        text: langue == "English" ? qsTr("Manage " + txt) : qsTr("管理 目标点")
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: closeBtn.right
        anchors.leftMargin: 11
        font.pointSize: 13
    }

    Button {
        id: createButton
        anchors {
            top: parent.top
            bottom: parent.bottom
            right: parent.right
        }
        anchors.rightMargin: 22

        width: Style.smallBtnWidth
        height: Style.smallBtnHeight

        background: Rectangle {
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            width: Math.min(createButton.width, createButton.height)
            height: createButton.width
            color: createButton.pressed ? Style.lightGreyBorder : createButton.hovered ? Style.lightGreyBackgroundHover : "transparent"
            radius: createButton.hovered ? createButton.width/2 : 0
        }

        Image {
            asynchronous: true
            source: "qrc:/icons/add"
            fillMode: Image.Pad // For not stretching image
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
        }
        onClicked: createMenu.open()

        CreatePointGroupPopupMenu {
            id: createMenu
            x: createButton.width
            langue: menuHeader.langue
            onOpenCreatePointMenu: menuHeader.openCreatePointMenu()
            onOpenCreateGroupMenu: menuHeader.openCreateGroupMenu()
        }
    }
}
