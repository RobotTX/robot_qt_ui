import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../Custom"

Frame {
    id: menuHeader
    padding: 0

    readonly property string txt: "Point"

    signal closeMenu(string txt)
    signal openCreatePointMenu()

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
        anchors.left: parent.left
        anchors.leftMargin: 11
    }

    Label {
        color: Style.midGrey2
        text: qsTr("Manage " + txt)
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: closeBtn.right
        anchors.leftMargin: 11
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
            color: "transparent"
        }

        Image {
            asynchronous: true
            source: "qrc:/icons/add"
            fillMode: Image.Pad // For not stretching image
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
        }
        onClicked: createMenu.open()

        Menu {
            id: createMenu
            x: createButton.width
            padding: 0
            width: 117

            background: Rectangle {
                implicitWidth: 117
                implicitHeight: 58
                color: Style.lightGreyBackground
                border.color: Style.lightGreyBorder
                radius: 5
            }

            MenuItem {
                text: qsTr("New Point")
                width: 117
                leftPadding: Style.menuItemLeftPadding
                height: Style.menuItemHeight
                onTriggered: openCreatePointMenu()
            }

            Rectangle {
                color: Style.lightGreyBorder
                width: 117
                height: 2
            }

            MenuItem {
                text: qsTr("New Group")
                width: 117
                leftPadding: Style.menuItemLeftPadding
                height: Style.menuItemHeight
            }
        }
    }
}
