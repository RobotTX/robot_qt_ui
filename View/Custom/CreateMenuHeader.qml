import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../Custom"

Frame {

    id: menuHeader

    property string langue

    padding: 0
    z: 2

    signal backToMenu()
    property string txt

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
        anchors.verticalCenter: parent.verticalCenter
        onClicked: backToMenu()
    }

    Label {
        color: Style.midGrey2
        text: langue == "English" ? qsTr("New " + txt) : qsTr("创建 " + txt)
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: closeBtn.right
        anchors.leftMargin: 11
    }
}
