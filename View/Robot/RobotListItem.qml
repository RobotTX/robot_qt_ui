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

    property variant consoleWhole: []
    property variant consoleWholeReverse: []
    property string consoleString: ""

    signal startDockingRobot(string ip)
    signal stopDockingRobot(string ip)
    signal rebootRobot(string ip)
    signal interruptDelay(string ip)

    signal soundOn(string ip)
    signal soundOff(string ip)

    height: 240 + robotPathListItem.height//105 + robotPathListItem.height
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
        tooltip: langue == "English" ? "退出程序" : "Shut Down Robot"

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
            property string ipToShutDown
            onTriggered: elapsed += interval;

            onElapsedChanged: {
                if(elapsed === restartButton.delay){
                    restartButton.released();
                    ipToShutDown = ip;
                    rebootRobotDialog.open();

                }
            }
        }

        CustomDialog {
            id: rebootRobotDialog
            parent: ApplicationWindow.overlay
            x: (parent.width - width) / 2
            y: (parent.height - height) / 2
            height: 130
            title: langue == "English" ? "警告"  : "Warning"
            message: "Do you want to power off robot " + name + " ?"
            acceptMessage: langue == "English" ? "是" : "Yes"
            rejectMessage: angue == "English" ? "取消" : "Cancel"
            onAccepted: {
                frame.rebootRobot(timerRestartButton.ipToShutDown);
            }
            onRejected: console.log("Cancel");
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
            tooltip: langue == "English" ? "取消" :"Cancel"
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
            tooltip: langue == "English" ? "保存" :"Save"
            imgSrc: "qrc:/icons/save"
            anchors {
                verticalCenter: nameField.verticalCenter
                right: parent.right
                rightMargin: 5
            }

            onClicked: {
                var newName = Helper.formatName(nameField.text);
                if(newName !== ""){
                    var NameLabel=""
                    if (langue == "English") {
                      NameLabel="机器名改成"
                    } else {
                      NameLabel="Robot renamed to"
                    }
                    nameLabel.visible = true;
                    nameField.focus = false;
                    robotModel.newNameSignal(ip, newName);
                    consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +NameLabel+ "\n\"" + newName+ "\" \n");
                    if (consoleWhole.length === 20) {
                        consoleWhole.splice(0,1);
                    }
                    robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
                    consoleString = consoleWholeReverse.join('');
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
        tooltip: if (charging == true) { langue == "English" ?"取消静音":"Unmute sound" } else {langue == "English" ?"静音":"Mute sound"}
        anchors {
            verticalCenter: nameLabel.verticalCenter
            right: rightButton.left
            leftMargin: 1
        }
        onReleased:  {
            var MuteLabel=""
            var UnmuteLabel=""
            if (langue == "English") {
                MuteLabel = "静音"
                UnmuteLabel = "取消静音"
            } else {
                MuteLabel = "Mute"
                UnmuteLabel = "Unmute"
            }
            charging = !charging;
            if (charging !== true) {
                frame.soundOn(ip);
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +UnmuteLabel+"\n");
            } else {
                // if mute is true then the corresponding command is x
                frame.soundOff(ip);
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +MuteLabel+ "\n");
            }
            if (consoleWhole.length === 20) {
                consoleWhole.splice(0,1);
            }
            robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
            consoleString = consoleWholeReverse.join('');
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
            robotModel: frame.robotModel
            langue: frame.langue
            onPointSelected: {
                var HomeLabel=""
                if (langue == "English") {
                  HomeLabel="新充电站"
                } else {
                  HomeLabel="New home"
                }
                robotModel.newHomeSignal(ip, _homeX, _homeY, orientation)
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +HomeLabel+ "\n");

                if (consoleWhole.length === 20) {
                    consoleWhole.splice(0,1);
                }
                robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
                consoleString = consoleWholeReverse.join('');
                }
            onPathSelected: {
                var ActionLabel=""
                var AssignLabel=""
                if (langue == "English") {
                    ActionLabel = "路径"
                    AssignLabel = "分配给机器"
                } else {
                    ActionLabel = "Path"
                    AssignLabel = "assigned to robot"
                }
                robotModel.newPathSignal(ip, _groupName, _pathName)
                frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +ActionLabel+ "\"" + _pathName + "\"" +AssignLabel+"\"\n"  + name +"\"\n");

                if (consoleWhole.length === 20) {
                    consoleWhole.splice(0,1);
                }
                robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
                consoleString = consoleWholeReverse.join('');
            }
            onRenameRobot: {
                nameLabel.visible = false;
                nameField.focus = true;
            }
            onDeletePath: {
                var PathLabel=""
                if (langue == "English") {
                  PathLabel="删除路径"
                } else {
                  PathLabel="Path deleted"
                }
                robotModel.deletePathSignal(ip)
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +PathLabel+ "\n");

                if (consoleWhole.length === 20) {
                    consoleWhole.splice(0,1);
                }

                robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
                consoleString = consoleWholeReverse.join('');
            }
            onLaserPressed: robotModel.activateLaser(ip, !laserActivated)
            onSaveCurrentPath: {
                var PathLabel=""
                var Savelabel=""
                if (langue == "English") {
                  PathLabel="当前的路径"
                  Savelabel="保存了"
                } else {
                    PathLabel="Current path"
                    Savelabel="saved"
                }
                pathModel.saveCurrentPath(pathName,pathPoints)
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +PathLabel+ "" + pathName + ""+Savelabel+"\n");

                if (consoleWhole.length === 20) {
                    consoleWhole.splice(0,1);
                }
                robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
                consoleString = consoleWholeReverse.join('');
            }
            onSaveCurrentHome: {
                if (homeOri < 0) {
                    homeOri = homeOri + 360;
                }
                var HomeLabel=""
                if (langue == "English") {
                  HomeLabel="当前的充电站保存了"
                } else {
                  HomeLabel="Current home saved"

                }

                pointModel.saveCurrentHome("CS", homeX, homeY, homeOri);
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +HomeLabel+"\n");

                if (consoleWhole.length === 20) {
                    consoleWhole.splice(0,1);
                }
                robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
                consoleString = consoleWholeReverse.join('');
            }
        }
    }

    Label {
        text: {
            if (robotModel.robotSelected === true && robotModel.getRobotIP() === ip) {
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Path \"" + robotModel.getPathName() + "\" \nassigned to robot \""  + robotModel.nameRobotPath +"\"\n");

                if (consoleWhole.length === 20) {
                    consoleWhole.splice(0,1);
                }
                robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
                consoleString = consoleWholeReverse.join('');
                robotModel.robotSelected = false;
            }
            qsTr("");
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
                    leftMargin: 2.2 * robotModel.getBatteryWarning(ip)
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
                text: langue == "English" ? "线速度" : "Linear Speed"
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
                text: qsTr(langue == "English" ? "角速度" : "Angular Speed")
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
            /// path deleted
            if (robotModel.deletePathButtonClicked) {
                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Path \"" + pathName + "\" \nhas been deleted\n");
                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                robotModel.msg = robotModel.inverseMsg.join('');
                langue == "English" ? qsTr("正在移动到 " + pathPoints.get(stage).pathPointName) : qsTr("Path \"" + pathName + "\" \nhas been deleted");
                robotModel.deletePathButtonClicked = false;
            }
            if (dockStatus === 3) {
                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Auto docking\n");
                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                robotModel.msg = robotModel.inverseMsg.join('');
                langue == "English" ? qsTr("正在移动到 " + pathPoints.get(stage).pathPointName) : qsTr("Auto docking");
            } else if (dockStatus === 0) {
                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Stop auto docking\n");
                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                robotModel.msg = robotModel.inverseMsg.join('');
                langue == "English" ? qsTr("正在移动到 " + pathPoints.get(stage).pathPointName) : qsTr("Stop auto docking");
            }

            if(pathName !== "" && pathPoints.count > 0){
                /// looping
                if (looping === true) {
                    langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Looping\n");
                    reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                    robotModel.msg = robotModel.inverseMsg.join('');
                    langue == "English" ? qsTr("正在移动到 " + pathPoints.get(stage).pathPointName) : qsTr("Looping");
                } else {
                    langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Unlooping\n");
                    reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                    robotModel.msg = robotModel.inverseMsg.join('');
                    langue == "English" ? qsTr("正在移动到 " + pathPoints.get(stage).pathPointName) : qsTr("Unlooping");
                }

                if(stage >= 0){
                    if(stage < pathPoints.count){
                        if(playingPath) {
                            if (stage == 0) {
                                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Heading to " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Robot is starting its \nmission" + "\n")
                                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                                robotModel.msg = robotModel.inverseMsg.join('');
                                langue == "English" ? qsTr("正在移动到 " + pathPoints.get(stage).pathPointName) : qsTr("Heading to " + pathPoints.get(stage).pathPointName);
                            } else {
                                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Heading to " + pathPoints.get(stage).pathPointName + "\n");
                                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                                robotModel.msg = robotModel.inverseMsg.join('');
                                langue == "English" ? qsTr("正在移动到 " + pathPoints.get(stage).pathPointName) : qsTr("Heading to " + pathPoints.get(stage).pathPointName);
                            }
                        } else {
                            if (robotModel.stopButtonClicked === true) {
                                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "等待移动到 " + pathPoints.get(stage).pathPointName + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Stopped robot \"" + name + "\" \nin its mission\n");
                                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                                robotModel.msg = robotModel.inverseMsg.join('');
                                langue == "English" ? qsTr("等待移动到 " + pathPoints.get(stage).pathPointName) : qsTr("Stopped robot in its mission");
                                robotModel.stopButtonClicked = false;
                            } else if (robotModel.pauseButtonClicked === true) {
                                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "等待移动到 " + pathPoints.get(stage).pathPointName + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Paused robot \"" + name + "\" \nin its mission\n");
                                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                                robotModel.msg = robotModel.inverseMsg.join('');
                                langue == "English" ? qsTr("等待移动到 " + pathPoints.get(stage).pathPointName) : qsTr("Paused robot in its mission");
                                robotModel.pauseButtonClicked = false;
                            } else if (dockStatus === 3) {
                                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Auto docking\n");
                                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                                robotModel.msg = robotModel.inverseMsg.join('');
                                langue == "English" ? qsTr("正在移动到 " + pathPoints.get(stage).pathPointName) : qsTr("Auto docking");
                            } else if (dockStatus === 0) {
                                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Stop auto docking\n");
                                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                                robotModel.msg = robotModel.inverseMsg.join('');
                                langue == "English" ? qsTr("正在移动到 " + pathPoints.get(stage).pathPointName) : qsTr("Stop auto docking");
                            }

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
                        robotModel.pathCompleted = true;
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
                            playingPath = false;
                            qsTr("Stuck going to " + pathPoints.get(Math.abs(stage + 1)).pathPointName);
//                            warningDialog.message = "Stuck going to " + pathPoints.get(Math.abs(stage + 1)).pathPointName;
//                            warningDialog.open();
                        } else {
                            langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "困在从 " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " 来的路上" + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Stuck going from " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " to " + pathPoints.get(Math.abs(stage + 1)).pathPointName + "\n");
                            reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                            robotModel.msg = robotModel.inverseMsg.join('');
                            langue == "English" ? qsTr("困在从 " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " 来的路上") : qsTr("Stuck going from " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " to " + pathPoints.get(Math.abs(stage + 1)).pathPointName);
                            playingPath = false;
                            qsTr("Stuck going from " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " to " + pathPoints.get(Math.abs(stage + 1)).pathPointName);
//                            warningDialog.message = "Stuck going from " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " to " + pathPoints.get(Math.abs(stage + 1)).pathPointName;
//                            warningDialog.open();
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
        parent: ApplicationWindow.overlay
        x: (parent.width - width) / 2
        y: (parent.height - height) / 2
        height: 60
        title: "Warning dialog"
    }


    RobotPathListItem {
        id: robotPathListItem
        robotModel: frame.robotModel
        pathModel: frame.pathModel
        langue: frame.langue
        consoleWhole: frame.consoleWhole
        consoleWholeReverse: frame.consoleWholeReverse
        consoleString: frame.consoleString
        anchors {
            top: pathLabel.bottom
            left: parent.left
            right: parent.right
            topMargin: 10
            leftMargin: 20
            rightMargin: 20
        }
        onPathSelected: {
            var ActionLabel=""
            var AssignLabel=""
            if (langue == "English") {
                ActionLabel = "路径"
                AssignLabel = "分配给机器"
            } else {
                ActionLabel = "Path"
                AssignLabel = "assigned to robot"
            }
            robotModel.newPathSignal(ip, _groupName, _pathName)
            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +ActionLabel+ "\"" + _pathName + "\"" +AssignLabel+"\""  + name +"\"\n");

            if (frame.consoleWhole.length === 20) {
                frame.consoleWhole.splice(0,1);
            }
            robotModel.reverse(frame.consoleWhole, frame.consoleWholeReverse, frame.consoleWhole.length);
            frame.consoleString = frame.consoleWholeReverse.join('');
        }
        onStartDockingRobot: {
            var ActionLabel=""
            var AutoDockingLabel=""
            if (langue == "English") {
                ActionLabel = "机器 ："
                AutoDockingLabel = "开始自动对接"
            } else {
                ActionLabel = "Start robot ："
                AutoDockingLabel = "auto docking process"
            }
            console.log("docking clicked !!!");
            frame.startDockingRobot(ip)
            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +ActionLabel+ "" + name + "\"\n"+AutoDockingLabel+"\n");
            if (frame.consoleWhole.length === 20) {
                frame.consoleWhole.splice(0,1);
            }

            robotModel.reverse(frame.consoleWhole, frame.consoleWholeReverse, frame.consoleWhole.length);

            frame.consoleString = frame.consoleWholeReverse.join('');

        }
        onStopDockingRobot: {
            var ActionLabel=""
            var AutoDockingLabel=""
            if (langue == "English") {
                ActionLabel = "机器 ："
                AutoDockingLabel = "停止自动对接"
            } else {
                ActionLabel = "Stop robot ："
                AutoDockingLabel = "auto docking process"
            }

            frame.stopDockingRobot(ip)
            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +ActionLabel+ "" + name + "\"\n"+AutoDockingLabel+"\n");
            if (frame.consoleWhole.length === 20) {
                frame.consoleWhole.splice(0,1);
            }

            robotModel.reverse(frame.consoleWhole, frame.consoleWholeReverse, frame.consoleWhole.length);

            frame.consoleString = frame.consoleWholeReverse.join('');
        }
        onInterruptDelay: frame.interruptDelay(ip)
        onStopPath: {
            var ActionLabel=""
            var MissonLabel=""
            if (langue == "English") {
                ActionLabel = "机器 ："
                MissonLabel = "停止行驶路径"
            } else {
                ActionLabel = "Stop robot ："
                MissonLabel = "misson"
            }
            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +ActionLabel+ "" + name + "\"\n"+MissonLabel+"\n");

            if (frame.consoleWhole.length === 20) {
                frame.consoleWhole.splice(0,1);
            }

            robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
            frame.consoleString = frame.consoleWholeReverse.join('');
        }
        onPlayPath: {
            var ActionLabel=""
            var MissonLabel=""
            if (langue == "English") {
                ActionLabel = "机器 ："
                MissonLabel = "暂停行驶路径"
            } else {
                ActionLabel = "Pause robot ："
                MissonLabel = "mission"
            }
            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +ActionLabel+ "" + name + "\"\n"+MissonLabel+"\n");

            if (frame.consoleWhole.length === 20) {
                frame.consoleWhole.splice(0,1);
            }

            robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
            frame.consoleString = frame.consoleWholeReverse.join('');
        }
        onPausePath: {
            var ActionLabel=""
            var MissonLabel=""
            if (langue == "English") {
                ActionLabel = "机器 ："
                MissonLabel = "开始行驶路径"
            } else {
                ActionLabel = "Start robot ："
                MissonLabel = "misson"
            }
            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +ActionLabel+ "" + name + "\"\n"+MissonLabel+"\n");

            if (frame.consoleWhole.length === 20) {
                frame.consoleWhole.splice(0,1);
            }

            robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
            frame.consoleString = frame.consoleWholeReverse.join('');
        }
        onLoopPath: {
            var LoopLabel=""
            if (langue == "English") {
                LoopLabel = "开始循环"
            } else {
                LoopLabel = "Looping"
            }
            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +LoopLabel+"\n");

            if (frame.consoleWhole.length === 20) {
                frame.consoleWhole.splice(0,1);
            }

            robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
            frame.consoleString = frame.consoleWholeReverse.join('');
        }
        onUnloopPath: {
            var LoopLabel=""
            if (langue == "English") {
                LoopLabel = "停止循环"
            } else {
                LoopLabel = "Unlooping"
            }
            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +LoopLabel+"\n");

            if (frame.consoleWhole.length === 20) {
                frame.consoleWhole.splice(0,1);
            }

            robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
            frame.consoleString = frame.consoleWholeReverse.join('');
        }
    }

    // console displaying actions
    Rectangle {
        id: idConsole
        visible: true
        height: 70
        anchors {
            top: robotPathListItem.bottom
            topMargin: 5
            left: parent.left
            right: parent.right
            leftMargin: 20
            rightMargin: 20
        }
        color: "white"
        border.width: 1
        border.color: Style.lightGreyBorder
        radius: 3

        Flickable {
            id: flickConsole
            ScrollBar.vertical: ScrollBar { }
            contentHeight: contentItem.childrenRect.height
            anchors.fill: parent
            clip: true

            Column {
                anchors.left: parent.left
                anchors.right: parent.right

                Text {
                    id: consoleMessage
                    fontSizeMode: Text.VerticalFit
                    wrapMode: Text.WordWrap
                    text: {
                        consoleString
                    }
                    font.pixelSize: 14
                    color: Style.midGrey2
                }
            }

        }
    }

    ToolSeparator {
        orientation: Qt.Horizontal

        contentItem: Rectangle {
            implicitHeight: 1
            color: Style.lightGreyBorder
        }

        anchors {
            bottom: parent.bottom
            left: parent.left
            right: parent.right
            leftMargin: 5
            rightMargin: 5
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
