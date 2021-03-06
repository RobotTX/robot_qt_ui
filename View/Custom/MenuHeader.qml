import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../Custom"

Frame {

    id: menuHeader
    property string langue

    padding: 0
    z: 2

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
        anchors.verticalCenter: parent.verticalCenter
        anchors.leftMargin: 11
    }

    Label {
        color: Style.midGrey2
        text: langue == "English" ? qsTr("Manage " + txt) : qsTr("管理 " + txt)
        font.pointSize: Style.ubuntuHeadingSize
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: closeBtn.right
        anchors.leftMargin: 11
    }
}
