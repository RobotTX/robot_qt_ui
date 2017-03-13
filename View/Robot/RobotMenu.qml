import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom"

Page {
    id: page
    anchors.fill: parent
    signal closeMenu()

    MenuHeader {
        id: robotMenuHeader
        objectName: "robotMenuHeader"
        txt: "Robot"
        onCloseMenu: page.closeMenu()
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

