import QtQuick 2.7
import QtQuick.Controls 2.0
import "../Custom"

Page {
    id: page
    readonly property int index: 0
    anchors.fill: parent

    MenuHeader {
        id: robotMenuHeader
        objectName: "robotMenuHeader"
        txt: "Robot"
    }

    RobotMenuContent {
        anchors {
            left: parent.left
            top: robotMenuHeader.bottom
            right: parent.right
            bottom: parent.bottom
        }
    }
}

