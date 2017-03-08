import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom"

Page {
    id: page
    readonly property int index: 1
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

