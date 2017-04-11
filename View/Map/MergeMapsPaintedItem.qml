import MergeMapsItems 1.0
import QtQuick 2.7
import QtQuick.Controls 2.1

MergeMapsPaintedItem {

    x: 0
    y: 0
    smooth: false

   onRotationChanged: console.log("rotation changed " + rotation)

    MouseArea {
        anchors.fill: parent
        drag.target: parent
        acceptedButtons: Qt.LeftButton
        onWidthChanged: console.log("width changed " + width);
        onHeightChanged: console.log("height changed " + height);
        onClicked: {
            console.log(width + " " + height)
            console.log("got clicked ");
        }
    }
}
