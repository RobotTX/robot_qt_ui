import QtQuick 2.7
import ScanMapsPaintedItem 1.0
import QtQuick.Controls 2.1
import "../Robot/"

ScanMapPaintedItem {

    id: item
    x: 0
    y: 0
    smooth: false

    MouseArea {
        anchors.fill: parent
        drag.target: parent
        acceptedButtons: Qt.LeftButton
        onWidthChanged: console.log("width changed scan item " + width);
        onHeightChanged: console.log("height changed scan item" + height);
        onClicked: {
            console.log(width + " " + height)
            console.log("got clicked " + parent.xRobot + " " + parent.yRobot + " " + parent.orientationRobot);
        }
    }

    Connections {
        target: item
        onUpdateRobot: {
            robotView.orientation = item.orientationRobot;
            robotView.x = item.xRobot;
            robotView.y = item.yRobot;
        }
    }


    // actually matters to create the mouse area of the robot after the mouse area of the item
    // in order for its events not to be stolen
    RobotView {
        id: robotView
        property real orientation: 0
        x: parent.xRobot
        y: parent.yRobot
    }
}
