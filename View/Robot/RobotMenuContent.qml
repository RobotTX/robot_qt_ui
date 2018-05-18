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
    property string langue

    signal startDockingRobot(string ip)
    signal stopDockingRobot(string ip)
    signal rebootRobot(string ip)
    signal soundOn(string ip)
    signal soundOff(string ip)
    signal setMessageTop(int status, string msg)
    signal interruptDelay(string ip)

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }
    padding: 0

    EmptyMenu {
        visible: robotModel.count === 0
        txt: langue == "English" ? "未链接机器人，请确保机器人和电脑在同一个WIFI网络" : "No robot connected. Make sure that the robot and your computer are connected to the same WIFI network."
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
                    langue: robotMenuFrame.langue
                    width: flick.width
                    onStartDockingRobot: robotMenuFrame.startDockingRobot(ip)
                    onStopDockingRobot: robotMenuFrame.stopDockingRobot(ip)
                    onRebootRobot: robotMenuFrame.rebootRobot(ip)
                    onSoundOn: robotMenuFrame.soundOn(ip)
                    onSoundOff: robotMenuFrame.soundOff(ip)
                    onSetMessageTop: robotMenuFrame.setMessageTop(status, msg)
                    onInterruptDelay: robotMenuFrame.interruptDelay(ip)
                }
            }
        }
    }

    Connections {
        target: robotModel
        onSoundOn: robotMenuFrame.soundOn(ip)
        onSoundOff: robotMenuFrame.soundOff(ip)
    }
}
