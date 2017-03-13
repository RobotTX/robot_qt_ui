import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../Custom"

Frame {
    id: menuHeader
    padding: 0

    property string txt

    signal closeMenu()

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
}
