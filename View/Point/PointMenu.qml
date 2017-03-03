import QtQuick 2.7
import QtQuick.Controls 2.0
import "../Custom"

Page {
    id: page
    anchors.fill: parent

    MenuHeader {
        id: pointMenuHeader
        objectName: "pointMenuHeader"
        txt: "Point"
    }

    PointMenuContent {
        anchors {
            left: parent.left
            top: pointMenuHeader.bottom
            right: parent.right
            bottom: parent.bottom
        }
    }
}


