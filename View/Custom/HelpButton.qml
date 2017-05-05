import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {

    property string tooltipText

    height: 20
    width: 20

    background: Rectangle {
        border.color: Style.lightGreyBorder
        border.width: 1
        radius: 10
    }

    ToolTip {
        visible: parent.hovered
        text: tooltipText
        font.pointSize: 10
        x: 26
        y: -4
        background: Rectangle {
            border.color: Style.darkSkyBlue
            border.width: 1
            radius: 8;
            anchors.fill: parent
        }
    }

    contentItem: Text {
        text: "?"
        font.pointSize: 12
        font.bold: true
        color: Style.darkSkyBlue
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
    }
}
