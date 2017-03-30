import MergeMapsItems 1.0
import QtQuick 2.7
import QtQuick.Controls 2.1

MergeMapsPaintedItem {

    x: 0
    y: 0
    smooth: false
    width: paintedItem._width
    height: paintedItem._height

    MouseArea {
        anchors.fill: parent
        drag.target: parent
        acceptedButtons: Qt.LeftButton
        onWidthChanged: console.log("width changed");
        onHeightChanged: console.log("height changed");
        onClicked: {
            console.log(width + " " + height)
            console.log("got clicked");
        }
    }
}
