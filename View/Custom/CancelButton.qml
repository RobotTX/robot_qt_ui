import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {
    property string txt: "Cancel"
    height: 23

    Label {
        text: qsTr(txt)
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
    }

    background: Rectangle {
        radius: 3
        border.width: 1
        border.color: Style.lightGreyBorder
    }
}
