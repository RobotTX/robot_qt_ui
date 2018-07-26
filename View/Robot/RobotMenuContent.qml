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
    property int homeXRobot
    property int homeYRobot
    property real batteryWarningThreshold
    property string langue

    signal startDockingRobot(string ip)
    signal stopDockingRobot(string ip)
    signal rebootRobot(string ip)
    signal soundOn(string ip)
    signal soundOff(string ip)
    signal setMessageTop(int status, string msg)
    signal interruptDelay(string ip)

    signal decreaseSound(string ip)
    signal increaseSound(string ip)


    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }
    padding: 0

    EmptyMenu {
        visible: robotModel.count === 0
        txt: langue == "English" ? "No robot connected.\nMake sure that the robot and your computer are connected to the same WIFI network." : "未连接机器人.\n请确保机器人和电脑在同一个WIFI网络"
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
                    homeXRobot: robotMenuFrame.homeXRobot
                    homeYRobot: robotMenuFrame.homeYRobot
                    width: flick.width
                    onStartDockingRobot: robotMenuFrame.startDockingRobot(ip)
                    onStopDockingRobot: robotMenuFrame.stopDockingRobot(ip)
                    onRebootRobot: robotMenuFrame.rebootRobot(ip)
                    onSoundOn: robotMenuFrame.soundOn(ip)
                    onSoundOff: robotMenuFrame.soundOff(ip)
               //     onSoundDecrease: robotMenuFrame.soundDecrease(ip)
                 //   onSoundIncrease: robotMenuFrame.soundIncrease(ip)
                    onSetMessageTop: robotMenuFrame.setMessageTop(status, msg)
                    onInterruptDelay: robotMenuFrame.interruptDelay(ip)
                    onDecreaseSound: robotMenuFrame.decreaseSound(ip)
                    onIncreaseSound: robotMenuFrame.increaseSound(ip)
                }
            }
        }
    }

    Connections {
        target: robotModel
        onSoundOn: robotMenuFrame.soundOn(ip)
        onSoundOff: robotMenuFrame.soundOff(ip)
        onDecreaseSound : robotMenuFrame.decreaseSound(ip)
        onIncreaseSound : robotMenuFrame.increaseSound(ip)
    }

    function homePosition(homeX, homeY) {
        console.log("we are in homePosition qml = " + homeX + " " + homeY);
        homeXRobot = homeX;
        homeYRobot = homeY;
    }
}
