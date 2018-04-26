import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../Guide"
import "../Custom"

Frame {
    id: menuHeader
    padding: 0
    z: 2
    property string txt
    property string langue
    property int menuIndex: 0
    signal closeMenu(string txt)
    signal closePath()

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
        onClicked: {
                menuHeader.closeMenu(txt)
        }
        imgSrc: "qrc:/icons/undo"
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: parent.left
        anchors.leftMargin: 11
    }

    Label {
        color: Style.midGrey2
//        text: langue == "English" ? qsTr("管理 目标点") : qsTr("Manage " + txt)
        text: qsTr(txt);
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: closeBtn.right
        anchors.leftMargin: 11
    }
}
