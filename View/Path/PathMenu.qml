import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom"
import "../../Model/Path"

Page {
    id: page
    anchors.fill: parent
    property Paths pathModel
    signal closeMenu()

    MenuHeader {
        id: pathMenuHeader
        objectName: "pathMenuHeader"
        txt: "Path"
        onCloseMenu: page.closeMenu()
    }

    PathMenuContent {
        pathModel: page.pathModel
        anchors {
            left: parent.left
            top: pathMenuHeader.bottom
            right: parent.right
            bottom: parent.bottom
        }
    }
}

