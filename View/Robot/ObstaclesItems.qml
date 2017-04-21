import QtQuick 2.0
import ObstaclesItems 1.0

ObstaclesPaintedItems {
    z: 3
    width: 600
    height: 600
    onXChanged: console.log("x " + x)
    onYChanged: console.log("y " + y)
    smooth: false
}
