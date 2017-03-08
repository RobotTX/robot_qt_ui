import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom"

Page {
    id: page
    readonly property int index: 3
    anchors.fill: parent

    MenuHeader {
        id: mapMenuHeader
        objectName: "mapMenuHeader"
        txt: "Map"
    }

    MapMenuContent {
        anchors {
            left: parent.left
            top: mapMenuHeader.bottom
            right: parent.right
            bottom: parent.bottom
        }
    }
}

