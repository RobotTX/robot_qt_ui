import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../../Model/Path"
import "../../Model/Robot"
import "../Custom"

Frame {
    id: frame
    property Points pointModel
    property Paths pathModel
    property Robots robotModel
    property real batteryWarningThreshold
    property string langue

    signal startDockingRobot(string ip)
    signal stopDockingRobot(string ip)
    signal rebootRobot(string ip)

    signal soundOn(string ip)
    signal soundOff(string ip)

    height: 225 + robotPathListItem.height//105 + robotPathListItem.height
    enabled: !processingCmd

    background: Rectangle {
        color: "transparent"
    }

    padding: 0

    SmallButton {
        id: restartButton
        anchors {
            top: parent.top
            left: parent.left
            leftMargin: 15
            topMargin: 15
        }
        width: 25
        height: 25

        imgSrc: "qrc:/icons/restart"
        tooltip: "Reboot the robot"

        property int delay: 1500
        onPressed: {
            console.log("timer starts")
            timerRestartButton.restart()
        }
        onReleased: {
            console.log("timer stop");
            timerRestartButton.stop();
            timerRestartButton.elapsed = 0
        }

        background: Rectangle {
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            width: Math.min(restartButton.width, restartButton.height)
            height: Math.min(restartButton.width, restartButton.height)
            color: restartButton.pressed ? Style.lightGreyBorder : restartButton.hovered ? Style.lightGreyBackgroundHover : restartButton.backColor
            radius: restartButton.hovered ? restartButton.width/2 : 0
            clip: true


            Rectangle {
                // draws a circle on top of the image whose radius grows as the timer's elapsed time increases
                id: timerBack
                z: 2
                width: restartButton.width * timerRestartButton.elapsed / restartButton.delay
                height: width
                color: Style.infoColor
                radius: timerBack.width/2
                anchors.verticalCenter: parent.verticalCenter
                anchors.horizontalCenter: parent.horizontalCenter
            }
        }

        Timer {
            id: timerRestartButton
            triggeredOnStart: false
            interval: 50
            repeat: true
            property int elapsed
            onTriggered: elapsed += interval;

            onElapsedChanged: {
                if(elapsed === restartButton.delay){
                    restartButton.released();
                    frame.rebootRobot(ip);
                }
            }
        }
    }

    CustomLabel {
        id: nameLabel
        text: qsTr(name)
        height: 20
        font.pixelSize: 16

        anchors {
            left: restartButton.right
            verticalCenter: restartButton.verticalCenter
//            right: rightButton.left
            right: muteButton.left
        }
    }

    Item {
        visible: !nameLabel.visible
        anchors {
            top: parent.top
            left: parent.left
            right: muteButton.left
            leftMargin: 20
            topMargin: 11
        }
        height: 28
        TextField {
            id: nameField
            selectByMouse: true
            placeholderText: qsTr(name)
            height: 28
            anchors {
                verticalCenter: parent.verticalCenter
                left: parent.left
                right: cancelName.left
                rightMargin: 5
            }
/*
            onEditingFinished: {
                if(saveName.enabled)
                    saveName.clicked()
            }
*/
            background: Rectangle {
                radius: 2

                border.color: nameField.text === "" ? Style.errorColor : nameField.activeFocus ? Style.lightBlue : Style.lightGreyBorder
                border.width: nameField.activeFocus || nameField.text === "" ? 3 : 1
            }
            onVisibleChanged: nameField.text = ""
        }

        SmallButton {
            id: cancelName
            tooltip: "Cancel"
            imgSrc: "qrc:/icons/closeBtn"
            anchors {
                verticalCenter: nameField.verticalCenter
                right: saveName.left
                rightMargin: 5
            }

            onClicked: {
                nameLabel.visible = true;
                nameField.focus = false;
            }
        }

        SmallButton {
            id: saveName
            tooltip: "Save"
            imgSrc: "qrc:/icons/save"
            anchors {
                verticalCenter: nameField.verticalCenter
                right: parent.right
                rightMargin: 5
            }

            onClicked: {
                var newName = Helper.formatName(nameField.text);
                if(newName !== ""){
                    nameLabel.visible = true;
                    nameField.focus = false;
                    robotModel.newNameSignal(ip, newName);
                }
            }
        }
    }

//    SmallButton {
//        id: batteryDisplay
//        objectName: "battery"
//        imgSrc:  "qrc:/icons/battery"
//        anchors {
//            verticalCenter: nameLabel.verticalCenter
//            right: rightButton.left
//            leftMargin: 5
//        }
//    }

    SmallButton {
        id: muteButton
        objectName: "muteBtn"
        imgSrc: if (charging == true) { "qrc:/icons/muteOn" } else  { "qrc:/icons/muteOff" }
        tooltip: if (charging == true) { "Unmute sound" } else {"Mute sound"}
        anchors {
            verticalCenter: nameLabel.verticalCenter
            right: rightButton.left
            leftMargin: 1
        }
        onReleased:  {
            charging = !charging;
            if (charging !== true) {
                frame.soundOn(ip);
            } else {
                // if mute is true then the corresponding command is x
                frame.soundOff(ip);
            }
        }
    }

    SmallButton {
        id: rightButton
        imgSrc: "qrc:/icons/more"
        anchors {
            verticalCenter: nameLabel.verticalCenter
            right: parent.right
            rightMargin: 15
        }

        onClicked: robotPopupMenu.open()

        RobotPopupMenu {
            id: robotPopupMenu
            x: rightButton.width
            pointModel: frame.pointModel
            pathModel: frame.pathModel
            langue: frame.langue
            onPointSelected: { console.log("robotModel.newHomeSignal");robotModel.newHomeSignal(ip, _homeX, _homeY, orientation)}
            onPathSelected: robotModel.newPathSignal(ip, _groupName, _pathName)
            onRenameRobot: {
                nameLabel.visible = false;
                nameField.focus = true;
            }
            onDeletePath: robotModel.deletePathSignal(ip)
            onLaserPressed: robotModel.activateLaser(ip, !laserActivated)
            onSaveCurrentPath: {
                console.log("in RobotListItem - onSaveCurrentPath pathName = " + pathName + " pathPoints = " + pathPoints)
                pathModel.saveCurrentPath(pathName,pathPoints)
                console.log("pos x = " + posX)
            }
            onSaveCurrentHome: {
                pointModel.saveCurrentHome("CS", homeX, homeY, homeOri);
                console.log("\n we are in onSaveCurrentHome - RobotListItem.qml");
                console.log( " home X = " + homeX + " home Y = " + homeY + " homeOri = " + homeOri);
            }
        }
    }

    ProgressBar {
        id: batteryLevel
        value: battery / 100

        anchors {
            top: nameLabel.bottom
            left: parent.left
            right: parent.right
            topMargin: 9
            leftMargin: 20
            rightMargin: 20
        }

        background: Rectangle {
            implicitWidth: parent.width - 10
            implicitHeight: 4

            color: Style.lightGreyBorder
            radius: 3
        }

        contentItem: Item {
            implicitWidth: parent.width
            implicitHeight: 4

            Rectangle {
                id: highlight
                width: batteryLevel.visualPosition * parent.width
                height: parent.height
                radius: 2
                color: battery < 50 * batteryWarningThreshold ? Style.errorColor2 : Style.darkSkyBlue
            }

            Rectangle {
                id: circleBluePoint
                height: 8
                width: height
                radius: height
                color: Style.darkSkyBlue
                anchors {
                    bottom: highlight.bottom
                    bottomMargin: -2
                    left: highlight.left
                    leftMargin: batteryLevel.visualPosition * parent.width - 3
                }
            }

            Rectangle {
                id: circle
                height: 8
                width: height
                radius: height
                color: Style.orangeColor
                anchors {
                    bottom: highlight.bottom
                    bottomMargin: -2
                    left: highlight.left
                    leftMargin: 2 * robotModel.getBatteryWarning(ip)
                }
            }

            Text {
                id: levelWarning
                anchors {
                    bottom: circle.top
                    left: circle.left
                    leftMargin: -2
                }
                color: Style.orangeColor
                text: Math.round(robotModel.getBatteryWarning(ip)) + "%"
                font.pixelSize: 11
            }

            Text {
                id: batteryValue
                anchors {
                     right: highlight.right;
                     rightMargin: - 15
                     top: highlight.bottom
                     topMargin: 1
                     verticalCenter: parent.verticalCenter
                }
                color: Style.darkSkyBlue
                text: battery + '%'
                font.pixelSize: 11
            }
        }
    }

    Rectangle {
        id: rectSpeed
        anchors {
            left: parent.left
            leftMargin: 20
            right: parent.right
            rightMargin: 20
            top : batteryLevel.bottom
            topMargin: 15
        }
        height: 35
        width: 100
        border.color: Style.lightGreyBorder
        border.width: 1
        radius: 3
        color: "white"

        RowLayout {
            id: rowSpeed
                width: 150
                height: 20

            Label {
                id: linearSpeedLabel
                text: "Linear Speed"
                anchors {
                    left: parent.left
                    leftMargin: 20
                }
                color: Style.midGrey2
                font.pointSize: 9
                horizontalAlignment: Text.AlignHCenter
            }


            Label {
                id: linearSpeed
                text: Math.round(linearVelocity * 100) / 100 + " m/s"
                color: Style.darkSkyBlue
                horizontalAlignment: Text.Center
                verticalAlignment: Text.Center
                font.pointSize: 9
                anchors {
                    top: linearSpeedLabel.bottom
                    left: linearSpeedLabel.left
                    leftMargin: 10
                    topMargin: 3
                }
            }

            Label {
                id: angularSpeedLabel
                text: qsTr("Angular Speed")
                anchors {
                    left: linearSpeedLabel.right
                    leftMargin: 40
                }
                font.pointSize: 9
                color: Style.midGrey2
                horizontalAlignment: Text.AlignHCenter
            }

            Label {
                id: angularSpeed
                text: Math.round(angularVelocity) + " deg/s"
                color: Style.darkSkyBlue
                horizontalAlignment: Text.Center
                verticalAlignment: Text.Center
                font.pointSize: 9
                anchors {
                    top: linearSpeed.top
                    left:angularSpeedLabel.left
                    leftMargin: 15
                }
            }
        }
    }

    property variant msgFinal: []

    function inverseArray(array1, array2, len) {
        console.log("array2 \n")
        for (var i = len - 1; i >= 0; i--) {
            for (var j = 0; j < len; j++) {
                array2[j] = array1[i];
                console.log(array2[j] + "\n");
            }
        }
    }

    signal setMessageTop(int status, string msg)

    function reverse(arr1, arr2, len) {
        for (var i = len-1; i >=0; i--) {
            arr2[(len-1) - i] = arr1[i];
//            console.log("\n in first loop normalArray["+i+"] = " + arr1[i] +  "\n secondArray == reverseArray["+((len-1) - i)+"] = " + arr2[(len-1)-i]);
        }
    }

    CustomLabel {
        id: pathLabel
        text: {
            if(pathName !== "" && pathPoints.count > 0){
                if(stage >= 0){
                    if(stage < pathPoints.count){
                        if(playingPath) {
                            if (stage == 0) {
                                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Heading to " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Robot is starting its mission" + "\n")
                                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                                robotModel.msg = robotModel.inverseMsg.join('');
                                langue == "English" ? qsTr("正在移动到 " + pathPoints.get(stage).pathPointName) : qsTr("Heading to " + pathPoints.get(stage).pathPointName);
                            } else {
                                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Heading to " + pathPoints.get(stage).pathPointName + "\n");
                                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                                robotModel.msg = robotModel.inverseMsg.join('');
//                                qsTr("Heading to " + pathPoints.get(stage).pathPointName);
                                langue == "English" ? qsTr("正在移动到 " + pathPoints.get(stage).pathPointName) : qsTr("Heading to " + pathPoints.get(stage).pathPointName);
                            }
                        } else {
                            langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "等待移动到 " + pathPoints.get(stage).pathPointName + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Waiting to go to " + pathPoints.get(stage).pathPointName + "\n");
                            reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                            robotModel.msg = robotModel.inverseMsg.join('');
                            langue == "English" ? qsTr("等待移动到 " + pathPoints.get(stage).pathPointName) : qsTr("Waiting to go to " + pathPoints.get(stage).pathPointName);
                        }
                    } else if (stage === pathPoints.count) {
                        langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "已完成当前路径" + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Current path completed" + "\n");
                        reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                        robotModel.msg = robotModel.inverseMsg.join('');
                        langue == "English" ? qsTr("已完成当前路径") : qsTr("Current path completed");

                    } else {
                        langue == "English" ? qsTr("路径目标点不存在当前状态") : qsTr("Stage not in the pathpoint list");
                    }
                } else {
                    if(Math.abs(stage + 1) < pathPoints.count){
                        if(stage == -1) {
                            langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "困在到 " + pathPoints.get(Math.abs(stage + 1)).pathPointName + "去的路上" + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Stuck going to " + pathPoints.get(Math.abs(stage + 1)).pathPointName + "\n");
                            reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                            robotModel.msg = robotModel.inverseMsg.join('');
                            langue == "English" ? qsTr("困在到 " +  pathPoints.get(Math.abs(stage + 1)).pathPointName + " 去的路上") : qsTr("Stuck going to " + pathPoints.get(Math.abs(stage + 1)).pathPointName);
                            warningDialog.message = "Stuck going to " + pathPoints.get(Math.abs(stage + 1)).pathPointName;
                            warningDialog.open();
                        } else {
                            langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "困在从 " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " 来的路上" + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Stuck going from " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " to " + pathPoints.get(Math.abs(stage + 1)).pathPointName + "\n");
                            reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                            robotModel.msg = robotModel.inverseMsg.join('');
                            langue == "English" ? qsTr("困在从 " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " 来的路上") : qsTr("Stuck going from " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " to " + pathPoints.get(Math.abs(stage + 1)).pathPointName);
                            warningDialog.message = "Stuck going from " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " to " + pathPoints.get(Math.abs(stage + 1)).pathPointName;
                            warningDialog.open();
                        }
                    } else {
                        langue == "English" ? qsTr("路径目标点不存在当前状态") : qsTr("Stage not in the pathpoint list");
                    }
                }
            } else {
                langue == "English" ? qsTr("尚未设置路径") : qsTr("No Path Assigned");
            }
        }
        font.pixelSize: 14
        color: {
            if(pathName !== "" && pathPoints.count > 0){
                if(stage >= 0)
                    Style.darkSkyBlue
                else
                    Style.errorColor2
            } else
                Style.midGrey2
        }
        anchors {
            top: rectSpeed.bottom
            left: parent.left
            right: parent.right
            topMargin: 10
            leftMargin: 20
            rightMargin: 20
        }
    }

    CustomDialog {
        id: warningDialog
        x: frame.x / 2
        y: frame.y / 2
        height: 60
        title: "Warning dialog"
    }


    RobotPathListItem {
        id: robotPathListItem
        robotModel: frame.robotModel
        pathModel: frame.pathModel
        langue: frame.langue
        anchors {
            top: pathLabel.bottom
            left: parent.left
            right: parent.right
            topMargin: 10
            leftMargin: 20
            rightMargin: 20
        }
        onPathSelected: robotModel.newPathSignal(ip, _groupName, _pathName)
        onStartDockingRobot: frame.startDockingRobot(ip)
        onStopDockingRobot: frame.stopDockingRobot(ip)
    }

    ToolSeparator {
        orientation: Qt.Horizontal
        anchors {
            bottom: parent.bottom
            left: parent.left
            right: parent.right
            leftMargin: 20
            rightMargin: 20
        }
    }

    Rectangle {
        visible: processingCmd
        radius: 3
        color: Style.lightGreyBackground
        opacity: 0.5
        anchors.fill: parent
    }

    CustomBusyIndicator {
        running: processingCmd
        visible: processingCmd
        anchors.fill: parent
    }


}
