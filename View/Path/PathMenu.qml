import QtQuick 2.7
import QtQuick.Controls 2.0
import "../Custom"

Page {
    id: page
    anchors.fill: parent

    MenuHeader {
        id: pathMenuHeader
        objectName: "pathMenuHeader"
        txt: "Path"
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

