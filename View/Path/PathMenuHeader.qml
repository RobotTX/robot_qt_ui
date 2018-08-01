import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../Custom"
import "../../Model/Path"

Frame {
    id: menuHeader
    padding: 0
    z: 2

    readonly property string txt: "Path"
    property Paths pathModel
    property string langue
    property bool test1 : false
    signal closeMenu(string txt)
    signal openCreatePathMenu()
    signal openCreateGroupMenu()

        QtObject{
            property bool test : false;
        }

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
        anchors.verticalCenter: parent.verticalCenter
    }

    Label {
        color: Style.midGrey2
        text: langue === "English" ? qsTr("Manage " + txt) : "管理 路径"
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: closeBtn.right
        anchors.leftMargin: 11
        font.pointSize: 16
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
        onClicked: createMenu.open();
        CreatePathGroupPopupMenu {
            id: createMenu
            x: createButton.width
            langue: menuHeader.langue
            pathModel: menuHeader.pathModel
            onOpenCreatePathMenu: menuHeader.openCreatePathMenu()
            onOpenCreateGroupMenu: menuHeader.openCreateGroupMenu()
        }
    }
}
