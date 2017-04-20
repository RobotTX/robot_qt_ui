import QtQuick 2.0
import ObstaclesItems 1.0

ObstaclesPaintedItems {
    Rectangle {
        border.color: "blue"
        border.width: 2
        color: "transparent"
        anchors.fill: parent
    }

    z: 3
    width: 600
    height: 600
    onXChanged: console.log("x " + x)
    onYChanged: console.log("y " + y)
    smooth: false
    Rectangle {
        color: "green"
        x: parent.width / 2
        y: parent.height / 2
        width: 1
        height: 1
    }
}
