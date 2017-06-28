import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../Custom"
import "../../Model/Point"
import "../../Model/Path"
import "../../Model/Robot"

Frame {
    id: robotMenuFrame
    objectName: "robotMenuFrame"
    property Robots robotModel
    property Points pointModel
    property Paths pathModel
    property real batteryWarningThreshold

    signal startDockingRobot(string ip)
    signal stopDockingRobot(string ip)
    signal rebootRobot(string ip)

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }
    padding: 0

    EmptyMenu {
        visible: robotModel.count === 0
        txt: "No robot connected. Make sure that the robot and your computer are connected to the same WIFI network."
        imgSrc: "qrc:/icons/big_robot"
    }

    Flickable {
        id: flick
        ScrollBar.vertical: ScrollBar { }
        contentHeight: contentItem.childrenRect.height
        anchors.fill: parent
        anchors.topMargin: 10

        Column {
            /// The list containing both the graphical and model of the robots in the menu
            Repeater {
                model: robotModel
                delegate: RobotListItem {
                    batteryWarningThreshold: robotMenuFrame.batteryWarningThreshold
                    pointModel: robotMenuFrame.pointModel
                    pathModel: robotMenuFrame.pathModel
                    robotModel: robotMenuFrame.robotModel
                    width: flick.width
                    onStartDockingRobot: robotMenuFrame.startDockingRobot(ip)
                    onStopDockingRobot: robotMenuFrame.stopDockingRobot(ip)
                    onRebootRobot: robotMenuFrame.rebootRobot(ip)
                }
            }
        }
    }
}
