import QtQuick 2.0
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

MenuItem {
    id: menuItem
    clip: true
    property string labelText

    background: Rectangle {
        anchors {
            fill: parent
            margins: 1
        }

        color: parent.hovered ? Style.lightGreyBackgroundHover : Style.lightGreyBackground
        radius: 5
    }

    contentItem: CustomLabel {
        color: enabled ? "black" : "lightgrey"
        text: qsTr(labelText)
        enabled: menuItem.enabled
        anchors {
            left: parent.left
            right: parent.right
            leftMargin: 20
            rightMargin: 5
            verticalCenter: parent.verticalCenter
        }
    }
}
