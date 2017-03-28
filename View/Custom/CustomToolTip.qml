import QtQuick 2.0
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

ToolTip {

    visible: parent.hovered

    background: Rectangle {
        anchors.fill: parent
        border.color: Style.lightGreyBorder
        radius: 10
    }
}
