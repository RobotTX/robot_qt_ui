import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom"

Page {
    id: page
    anchors.fill: parent
    signal closeMenu()

    MenuHeader {
        id: pathMenuHeader
        objectName: "pathMenuHeader"
        txt: "Path"
        onCloseMenu: page.closeMenu()
    }

    PathMenuContent {
        anchors {
            left: parent.left
            top: pathMenuHeader.bottom
            right: parent.right
            bottom: parent.bottom
        }
    }
}

