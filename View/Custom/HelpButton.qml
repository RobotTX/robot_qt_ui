import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {
    id: helpButton
    property string tooltipText

    height: 25
    width: 25

    background: Rectangle {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        width: Math.min(helpButton.width, helpButton.height)
        height: Math.min(helpButton.width, helpButton.height)
        border.color: Style.lightGreyBorder
        color: helpButton.pressed ? Style.lightGreyBorder : helpButton.hovered ? Style.lightGreyBackgroundHover : "white"
        radius: helpButton.width/2
    }

    ToolTip {
        id: tooltip
        visible: parent.hovered && tooltipText !== ""
        text: tooltipText
        font.pointSize: 10
        x: helpButton.width + 5
        y: (helpButton.width - tooltip.height) / 2

        background: Rectangle {
            border.color: Style.darkSkyBlue
            border.width: 1
            radius: 8
            anchors.fill: parent
        }
    }

    contentItem: Text {
        text: "?"
        font.pointSize: 14
        font.bold: true
        color: Style.darkSkyBlue
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
    }
}
