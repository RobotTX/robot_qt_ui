import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../Guide"
import "../Custom"

Frame {
    id: menuHeader
    padding: 0
    z: 2
    readonly property string txt: "Guide"
    property string langue
    signal closeMenu(string txt)
    signal openCreateSpeechMenu()
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
        text: langue == "English" ? qsTr("管理 目标点") : qsTr("Manage " + txt)
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: closeBtn.right
        anchors.leftMargin: 11
    }
}
