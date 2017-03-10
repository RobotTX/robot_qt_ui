import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {
    height: 23

    Label {
        text: qsTr("Save")
        color: "white"
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
    }

    background: Rectangle {
        radius: 3
        color: enabled ? (pressed ? Style.darkSkyBlueBorder : Style.darkSkyBlue) : Style.disableSaveColor
        border.width: 1
        border.color: enabled ? Style.darkSkyBlueBorder : Style.disableSaveBorder
    }
}
