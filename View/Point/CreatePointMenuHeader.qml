import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../Custom"

Frame {
    id: menuHeader
    padding: 0

    signal backToMenu()

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
        imgSrc: "qrc:/icons/back"
        anchors.left: parent.left
        anchors.leftMargin: 11
        onClicked: backToMenu()
    }

    Label {
        color: Style.midGrey2
        text: qsTr("New Point")
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: closeBtn.right
        anchors.leftMargin: 11
    }
}
