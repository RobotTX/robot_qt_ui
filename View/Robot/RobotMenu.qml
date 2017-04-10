import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom"
import "../../Model/Robot"
import "../../Model/Point"
import "../../Model/Path"

Page {
    id: page
    anchors.fill: parent

    property Robots robotModel
    property Points pointModel
    property Paths pathModel
    property real batteryWarningThreshold
    signal useRobotPathModel(bool use)
    signal closeMenu()

    onVisibleChanged: {
        if(visible){
            useRobotPathModel(true);
            pathModel.visiblePathChanged();
        }
    }

    MenuHeader {
        id: robotMenuHeader
        txt: "Robot"
        onCloseMenu: page.closeMenu()
    }

    RobotMenuContent {
        robotModel: page.robotModel
        pointModel: page.pointModel
        pathModel: page.pathModel
        batteryWarningThreshold: page.batteryWarningThreshold
        anchors {
            left: parent.left
            top: robotMenuHeader.bottom
            right: parent.right
            bottom: parent.bottom
        }
    }
}

