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
    property string langue

    signal setMessageTop(int status, string msg)
    signal useRobotPathModel(bool use)
    signal closeMenu()

    onVisibleChanged: {
        if(visible){
            useRobotPathModel(true);
            pathModel.setPathVisible(); /// setPathIsVisible to false so in RobotView we don't see them
            pathModel.visiblePathChanged();
        } else {
            useRobotPathModel(false);
        }
    }

    MenuHeader {
        id: robotMenuHeader
        txt: langue == "English" ? "机器人" : "Robot"
        langue: page.langue
        onCloseMenu: page.closeMenu()
    }

    RobotMenuContent {
        robotModel: page.robotModel
        pointModel: page.pointModel
        pathModel: page.pathModel
        langue: page.langue
        onSetMessageTop: page.setMessageTop(status, msg)
        batteryWarningThreshold: page.batteryWarningThreshold
        anchors {
            left: parent.left
            top: robotMenuHeader.bottom
            right: parent.right
            bottom: parent.bottom
        }
    }
}

