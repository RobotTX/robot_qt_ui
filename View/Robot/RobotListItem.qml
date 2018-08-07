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
    property string homeXRobot
    property string homeYRobot
    property real batteryWarningThreshold
    property string langue
    property int menuIndex: -1

    property variant consoleWhole: []
    property variant consoleWholeReverse: []
    property string consoleString: ""

    signal startDockingRobot(string ip)
    signal stopDockingRobot(string ip)
    signal rebootRobot(string ip)
    signal interruptDelay(string ip)

    signal soundOn(string ip)
    signal soundOff(string ip)
    signal decreaseSound(string ip)
    signal increaseSound(string ip)
    signal magnetLock(string ip)

    height: 260 + robotPathListItem.height//105 + robotPathListItem.height
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
        tooltip: langue == "English" ? "Shut Down Robot" : "关机"

        property int delay: 700
        onPressed: {
            timerRestartButton.restart()
        }
        onReleased: {
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
            font.pointSize: Style.ubuntuSubHeadingSize
            x: (parent.width - width) / 2
            y: (parent.height - height) / 2
            height: 130
            topMarginLabel: langue === "English" ? 10 : 20;
            leftMarginLabel: langue === "English" ? 35 : 90;
            colorBackground: "white"
            title: langue == "English" ? "WARNING"  : "警告"
            message: langue == "English" ? "\nDo you want to power off robot " + name + "?" : "你想关闭机器人 " + name + "吗?"
            acceptMessage: langue == "English" ? "Yes" : "确认"
            rejectMessage: langue == "English" ? "Cancel" : "取消"

            onAccepted: {
                frame.rebootRobot(timerRestartButton.ipToShutDown);
            }
            onRejected: {}
        }
    }

    CustomLabel {
        id: nameLabel
        text: qsTr(name)
        height: 20
        font.pointSize: Style.ubuntuHeadingSize

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
//            height: langue === "English" ? 35 : 28
            verticalAlignment: TextInput.AlignVCenter
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
            tooltip: langue == "English" ? "Cancel" :"取消"
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
            tooltip: langue == "English" ? "Save" :"保存"
            imgSrc: "qrc:/icons/save"
            anchors {
                verticalCenter: nameField.verticalCenter
                right: parent.right
                rightMargin: 5
            }

            onClicked: {
                var newName = Helper.formatName(nameField.text);
                if(newName !== ""){
                    var nameLabelChange = ""
                    var msg = ""
                    if (langue == "English") {
                      nameLabelChange = "Robot renamed to"
                    } else {
                      nameLabelChange ="机器名改成"
                    }
                    nameLabel.visible = true;
                    nameField.focus = false;
                    robotModel.newNameSignal(ip, newName);
                    setMessageTop(2, nameLabelChange + "\"" + newName + "\"");
                    consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + nameLabelChange + "\n\"" + newName+ "\" \n");
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
        tooltip: if (charging == true) { langue == "English" ?"Unmute sound":"取消静音" } else {langue == "English" ?"Mute sound":"静音"}
        anchors {
            verticalCenter: nameLabel.verticalCenter
            right: rightButton.left
            leftMargin: 1
        }
        onReleased:  {
            var muteLabel=""
            var unmuteLabel=""
            if (langue == "English") {
                muteLabel = "Mute"
                unmuteLabel = "Unmute"
            } else {
                muteLabel = "静音"
                unmuteLabel = "取消静音"
            }
            charging = !charging;
            if (charging !== true) {
                frame.soundOn(ip);
                setMessageTop(2, unmuteLabel);
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +unmuteLabel+"\n");
            } else {
                // if mute is true then the corresponding command is x
                frame.soundOff(ip);
                setMessageTop(2, muteLabel);
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +muteLabel+ "\n");
            }
            if (consoleWhole.length === 20) {
                consoleWhole.splice(0,1);
            }
            robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
            consoleString = consoleWholeReverse.join('');
        }
    }
//    SmallButton{
//        id : increaseButton
//        imgSrc: "qrc:/icons/increaseBtn"
//            anchors{
//                top: muteButton.bottom
//                right : decreaseButton.left
//                topMargin: muteButton.bottom

//        }
//            onReleased: {
//                robotModel.increaseSound(ip);
//            }

//    }
//     SmallButton{
//        id : decreaseButton
//        imgSrc :"qrc:/icons/decreaseBtn"
//            anchors{
//            right: parent.right

//            verticalCenter: increaseButton.verticalCenter
//            //right : parent.right
//             rightMargin : 15
//            leftMargin: 1
//            //left: increaseButton.right

//        }
//            onClicked:{
//               robotModel.decreaseSound(ip);
//            }

//    }

    SmallButton {
        id: increaseButton
        objectName: "decreaseSound"
        imgSrc: "qrc:/icons/decrease10"
        tooltip: langue === "English" ? "Decrease" : "减小音量"

        anchors {
            top: muteButton.bottom
            right: muteButton.right
            rightMargin: 8
        }
        onReleased:  {
            frame.decreaseSound(ip);
        }
    }

    SmallButton {
        id: decreaseButton
        objectName: "increaseSound"
        imgSrc: "qrc:/icons/increase10"
        tooltip: langue === "English" ? "Increase" : "增大音量"
        anchors {
            top: muteButton.bottom
            left: muteButton.left
            leftMargin: 10

        }
        onReleased:  {
            frame.increaseSound(ip);
        }
    }
    SmallButton{
        id: magnetButton
        imgSrc: "qrc:/icons/lockTrail"
        tooltip: langue === "English" ? "Detach Object" : "分离目标"
        visible: lockTrail != false
        anchors{
            verticalCenter: nameLabel.verticalCenter
            right: muteButton.left
            rightMargin: 5
        }
        onReleased: {
            frame.magnetLock(ip);
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

        onClicked: {
            menuIndex = -1;
            robotPopupMenu1.open();

        }

        RobotPopupMenu {
            id: robotPopupMenu1
            x: rightButton.width
            currentMenuIndex: frame.menuIndex
            pointModel: frame.pointModel
            pathModel: frame.pathModel
            robotModel: frame.robotModel
            langue: frame.langue
            onDoNothing: {
                menuIndex = -1;
                robotPopupMenu2.open();
            }

            onPointSelected: {
                var homeLabel=""
                if (langue == "English") {
                  homeLabel="New home"
                } else {
                  homeLabel="新充电站"
                }
                setMessageTop(2, homeLabel);
                robotModel.newHomeSignal(ip, _homeX, _homeY, orientation)
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + homeLabel+ "\n");

                if (consoleWhole.length === 20) {
                    consoleWhole.splice(0,1);
                }
                robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
                consoleString = consoleWholeReverse.join('');
                }
            onPathSelected: {
                var actionLabel=""
                var assignLabel=""
                if (langue == "English") {
                    actionLabel = "Path"
                    assignLabel = "assigned to robot"
                } else {
                    actionLabel = "路径"
                    assignLabel = "分配给机器"

                }

                setMessageTop(2, actionLabel+ "\"" + _pathName + "\"" + assignLabel + "\""  + name +"\"");
                robotModel.newPathSignal(ip, _groupName, _pathName)
                frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +actionLabel+ "\"" + _pathName + "\n\"" + assignLabel + "\"" + name +"\"\n");

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
                var pathLabel=""
                if (langue == "English") {
                   pathLabel="Path deleted"
                } else {
                  pathLabel="删除路径"
                }
                setMessageTop(2, pathLabel);
                robotModel.deletePathSignal(ip)
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + pathLabel + "\n");

                if (consoleWhole.length === 20) {
                    consoleWhole.splice(0,1);
                }

                robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
                consoleString = consoleWholeReverse.join('');
            }
            onLaserPressed: robotModel.activateLaser(ip, !laserActivated)
            onSaveCurrentPath: {
                var pathLabel=""
                var savelabel=""
                if (langue == "English") {
                    pathLabel="Current path"
                    savelabel="saved"
                } else {
                    pathLabel="当前的路径"
                    savelabel="保存了"
                }

                setMessageTop(2, pathLabel + " " + pathName + " " + savelabel)
                pathModel.saveCurrentPath(pathName,pathPoints)
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + pathLabel + "\"" + pathName + "\"\n"+ savelabel + "\n");

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
                var homeLabel=""
                if (langue == "English") {
                  homeLabel="Saved current home"

                } else {
                  homeLabel="已保存当前充电站"

                }

                setMessageTop(2, homeLabel)
                pointModel.saveCurrentHome("CS", homeX, homeY, homeOri);
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + homeLabel + "\n");

                if (consoleWhole.length === 20) {
                    consoleWhole.splice(0,1);
                }
                robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
                consoleString = consoleWholeReverse.join('');
            }
        }

        /// we actually duplicate previous RobotPopupMenu in order to open again the popup
        /// when clicking on one item. this is a little ugly but for now only solution found
        RobotPopupMenu {
            id: robotPopupMenu2
            x: rightButton.width
            currentMenuIndex: frame.menuIndex
            pointModel: frame.pointModel
            pathModel: frame.pathModel
            robotModel: frame.robotModel
            langue: frame.langue
            onDoNothing: {
                menuIndex = -1;
                robotPopupMenu1.open();
            }

            onPointSelected: {
                var homeLabel=""
                if (langue == "English") {
                  homeLabel="New home"
                } else {
                  homeLabel="新充电站"
                }
                setMessageTop(2, homeLabel);
                robotModel.newHomeSignal(ip, _homeX, _homeY, orientation)
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + homeLabel+ "\n");

                if (consoleWhole.length === 20) {
                    consoleWhole.splice(0,1);
                }
                robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
                consoleString = consoleWholeReverse.join('');
                }
            onPathSelected: {
                var actionLabel=""
                var assignLabel=""
                if (langue == "English") {
                    actionLabel = "Path"
                    assignLabel = "assigned to robot"
                } else {
                    actionLabel = "路径"
                    assignLabel = "分配给机器"
                }

                setMessageTop(2, actionLabel+ "\"" + _pathName + "\"" + assignLabel + "\""  + name +"\"");
                robotModel.newPathSignal(ip, _groupName, _pathName)
                frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " +actionLabel+ "\"" + _pathName + "\n\"" + assignLabel + "\"" + name +"\"\n");

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
                var pathLabel=""
                if (langue == "English") {
                  pathLabel="Path deleted"
                } else {
                  pathLabel="删除路径"
                }
                setMessageTop(2, pathLabel);
                robotModel.deletePathSignal(ip)
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + pathLabel + "\n");

                if (consoleWhole.length === 20) {
                    consoleWhole.splice(0,1);
                }

                robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
                consoleString = consoleWholeReverse.join('');
            }
            onLaserPressed: robotModel.activateLaser(ip, !laserActivated)
            onSaveCurrentPath: {
                var pathLabel=""
                var savelabel=""
                if (langue == "English") {
                    pathLabel="Current path"
                    savelabel="saved"
                } else {
                    pathLabel="当前的路径"
                    savelabel="保存了"
                }

                setMessageTop(2, pathLabel + " " + pathName + " " + savelabel)
                pathModel.saveCurrentPath(pathName,pathPoints)
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + pathLabel + "\"" + pathName + "\"\n"+ savelabel + "\n");

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
                var homeLabel=""
                if (langue == "English") {
                  homeLabel="Saved current home"
                } else {
                  homeLabel="已保存当前充电站"
                }

                setMessageTop(2, homeLabel)
                pointModel.saveCurrentHome("CS", homeX, homeY, homeOri);
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + homeLabel + "\n");

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
            var actionLabel=""
            var assignLabel=""
            if (langue == "English") {
                actionLabel = "Path"
                assignLabel = "assigned to robot"
            } else {
                actionLabel = "路径"
                assignLabel = "分配给机器"
            }

            if (robotModel.robotSelected === true && robotModel.getRobotIP() === ip) {
                consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "\"" + actionLabel + "\"" + robotModel.getPathName() + "\n\"" + assignLabel + "\""  + robotModel.nameRobotPath +"\"\n");

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

            top: increaseButton.bottom

            left: parent.left
            right: parent.right
//            topMargin: 9
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
                font.pointSize: Style.ubuntuSubHeadingSize
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
                font.pointSize: Style.ubuntuTextSize
            }
        }
    }

    Rectangle {
        id: rectSpeed
        anchors {
            left: parent.left
            leftMargin: 10
            right: parent.right
            rightMargin: 20
            top : batteryLevel.bottom
            topMargin: 15
        }
        height: 50
        width: 100
//        border.color: Style.lightGreyBorder
//        border.width: 1
        radius: 3
        color: "transparent"

        RowLayout {
            id: rowSpeed
                width: parent.width
                height: 20

            Image {
                id: linearSpeedLabel
                asynchronous: true
                fillMode: Image.Pad
                source: "qrc:/icons/speed_linear"
                anchors {
                    left: parent.left
                    leftMargin: 35
                }
            }

            Label {
                id: linearSpeed
                text: Math.round(linearVelocity * 100) / 100 + " m/s"
                color: Style.darkSkyBlue
                horizontalAlignment: Text.Center
                verticalAlignment: Text.Center
                font.pointSize: Style.ubuntuTextSize
                anchors {
                    top: linearSpeedLabel.bottom
                    left: linearSpeedLabel.left
//                    leftMargin: langue === "English" ? -5 : 10
                    leftMargin: 0
                    topMargin: 3
                }
            }

            Image {
                id: angularSpeedLabel
                asynchronous: true
                fillMode: Image.Pad
                source: "qrc:/icons/speed_angular"
                anchors {
                    right: parent.right
                    rightMargin: 35
                }
            }

            Label {
                id: angularSpeed
                text: Math.round(angularVelocity) + " deg/s"
                color: Style.darkSkyBlue
                horizontalAlignment: Text.Center
                verticalAlignment: Text.Center
                font.pointSize: Style.ubuntuTextSize
                anchors {
                    top: linearSpeed.top
                    left:angularSpeedLabel.left
//                    leftMargin: langue === "English" ? -10 : 15
                    leftMargin: 0
                }
            }
        }
    }

    property variant msgFinal: []

    signal setMessageTop(int status, string msg)

    function reverse(arr1, arr2, len) {
        for (var i = len-1; i >=0; i--) {
            arr2[(len-1) - i] = arr1[i];
        }
    }

    CustomLabel {
        id: pathLabel
        text: {
            /// path deleted
            if (robotModel.deletePathButtonClicked) {
                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Path \"" + pathName + "\" \nhas been deleted\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n");
                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                robotModel.msg = robotModel.inverseMsg.join('');
                langue == "English" ? qsTr("Path \"" + pathName + "\" \nhas been deleted") : qsTr("正在移动到 " + pathPoints.get(stage).pathPointName);
                robotModel.deletePathButtonClicked = false;
            }
            if (dockStatus === 3) {
                langue == "English" ?  robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Auto docking\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n");
                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                robotModel.msg = robotModel.inverseMsg.join('');
                langue == "English" ? qsTr("Auto docking") : qsTr("正在移动到 " + pathPoints.get(stage).pathPointName);
            } else if (dockStatus === 0) {
                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Stop auto docking\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n");
                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                robotModel.msg = robotModel.inverseMsg.join('');
                langue == "English" ? qsTr("Stop auto docking") : qsTr("正在移动到 " + pathPoints.get(stage).pathPointName);
            }

            if(pathName !== "" && pathPoints.count > 0){
                /// looping
                if (looping === true) {
                    langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Looping\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n");
                    reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                    robotModel.msg = robotModel.inverseMsg.join('');
                    langue == "English" ? qsTr("Looping") : qsTr("正在移动到 " + pathPoints.get(stage).pathPointName);
                } else {
                    langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Unlooping\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n");
                    reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                    robotModel.msg = robotModel.inverseMsg.join('');
                    langue == "English" ? qsTr("Unlooping") : qsTr("正在移动到 " + pathPoints.get(stage).pathPointName);
                }

                if(stage >= 0){
                    if(stage < pathPoints.count){
                        if(playingPath) {
                            if (stage === 0) {
                                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Heading to " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Robot is starting its \nmission" + "\n"): robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n")
                                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                                robotModel.msg = robotModel.inverseMsg.join('');
                                langue == "English" ? qsTr("Heading to " + pathPoints.get(stage).pathPointName) : qsTr("正在移动到 " + pathPoints.get(stage).pathPointName);
                            } else {
                                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Heading to " + pathPoints.get(stage).pathPointName + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n");
                                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                                robotModel.msg = robotModel.inverseMsg.join('');
                                langue == "English" ? qsTr("Heading to " + pathPoints.get(stage).pathPointName) : qsTr("正在移动到 " + pathPoints.get(stage).pathPointName);
                            }
                        } else {
                            if (robotModel.stopButtonClicked === true) {
                                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Stopped robot \"" + name + "\" \nin its mission\n"): robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "等待移动到 " + pathPoints.get(stage).pathPointName + "\n");
                                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                                robotModel.msg = robotModel.inverseMsg.join('');
                                langue == "English" ? qsTr("Stopped robot in its mission") : qsTr("等待移动到 " + pathPoints.get(stage).pathPointName);
                                robotModel.stopButtonClicked = false;
                            } else if (robotModel.pauseButtonClicked === true) {
                                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Paused robot \"" + name + "\" \nin its mission\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "等待移动到 " + pathPoints.get(stage).pathPointName + "\n");
                                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                                robotModel.msg = robotModel.inverseMsg.join('');
                                langue == "English" ? qsTr("Paused robot in its mission") : qsTr("等待移动到 " + pathPoints.get(stage).pathPointName);
                                robotModel.pauseButtonClicked = false;
                            } else if (dockStatus === 3) {
                                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Auto docking\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n");
                                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                                robotModel.msg = robotModel.inverseMsg.join('');
                                langue == "English" ? qsTr("Auto docking") : qsTr("正在移动到 " + pathPoints.get(stage).pathPointName);
                            } else if (dockStatus === 0) {
                                langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Stop auto docking\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "正在移动到 " + pathPoints.get(stage).pathPointName + "\n" + Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "机器人开始执行任务" + "\n");
                                reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                                robotModel.msg = robotModel.inverseMsg.join('');
                                langue == "English" ? qsTr("Stop auto docking") : qsTr("正在移动到 " + pathPoints.get(stage).pathPointName);
                            }

                            langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Waiting to go to " + pathPoints.get(stage).pathPointName + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "等待移动到 " + pathPoints.get(stage).pathPointName + "\n");
                            reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                            robotModel.msg = robotModel.inverseMsg.join('');
                            langue == "English" ? qsTr("Waiting to go to " + pathPoints.get(stage).pathPointName) : qsTr("等待移动到 " + pathPoints.get(stage).pathPointName);
                        }
                    } else if (stage === pathPoints.count) {
                        langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Current path completed" + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "已完成当前路径" + "\n");
                        reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                        robotModel.msg = robotModel.inverseMsg.join('');
                        langue == "English" ? qsTr("Current path completed") : qsTr("已完成当前路径");
                        robotModel.pathCompleted = true;
                    } else {
                        langue == "English" ? qsTr("Stage not in the pathpoint list") : qsTr("路径目标点不存在当前状态");
                    }
                } else {
                    if(Math.abs(stage + 1) < pathPoints.count){

                        if(stage == -1) {
                            playingPath = false;
                            langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Stuck going to " + pathPoints.get(Math.abs(stage + 1)).pathPointName + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "困在到 " + pathPoints.get(Math.abs(stage + 1)).pathPointName + "去的路上" + "\n");
                            reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                            robotModel.msg = robotModel.inverseMsg.join('');
                            langue == "English" ? qsTr("Stuck going to " + pathPoints.get(Math.abs(stage + 1)).pathPointName) : qsTr("困在到 " +  pathPoints.get(Math.abs(stage + 1)).pathPointName + " 去的路上");
//                            warningDialog.message = "Stuck going to " + pathPoints.get(Math.abs(stage + 1)).pathPointName;
//                            warningDialog.open();
                        } else {
                            playingPath = false;
                            langue == "English" ? robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "Stuck going from " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " to " + pathPoints.get(Math.abs(stage + 1)).pathPointName + "\n") : robotModel.msgs.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + "困在从 " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " 来的路上" + "\n");
                            reverse(robotModel.msgs,robotModel.inverseMsg, robotModel.msgs.length, robotModel.inverseMsg.length)
                            robotModel.msg = robotModel.inverseMsg.join('');
                            langue == "English" ? qsTr("Stuck going from " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " to " + pathPoints.get(Math.abs(stage + 1)).pathPointName) : qsTr("困在从 " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " 来的路上");
                            qsTr("Stuck going from " + pathPoints.get(Math.abs(stage  + 2)).pathPointName + " to " + pathPoints.get(Math.abs(stage + 1)).pathPointName);
                        }
                    } else {
                        langue == "English" ? qsTr("Stage not in the pathpoint list") : qsTr("路径目标点不存在当前状态");
                    }
                }
            } else {
                langue == "English" ? qsTr("No Path Assigned") : qsTr("尚未设置路径");
            }
        }
        font.pointSize: Style.ubuntuSubHeadingSize
        color: {
            if(pathName !== "" && pathPoints.count > 0){
                if(stage >= 0) {
                    Style.darkSkyBlue
                } else {
                    playingPath = false;
                    Style.errorColor2
                }
            } else {
                Style.midGrey2
            }
        }
        anchors {
            top: rectSpeed.bottom
            left: parent.left
            right: parent.right
            topMargin: 25
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
        homeXRobot: frame.homeXRobot
        homeYRobot: frame.homeYRobot
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
            var actionLabel=""
            var assignLabel=""
            if (langue == "English") {
                actionLabel = "Path "
                assignLabel = "assigned to robot"
            } else {
                actionLabel = "路径"
                assignLabel = "分配给机器"
            }

            setMessageTop(2, actionLabel + "\"" + _pathName + "\"" + assignLabel + "\"" + name);

            robotModel.newPathSignal(ip, _groupName, _pathName)
            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + actionLabel + "\"" + _pathName + "\"" + assignLabel +"\""  + name +"\"\n");

            if (frame.consoleWhole.length === 20) {
                frame.consoleWhole.splice(0,1);
            }
            robotModel.reverse(frame.consoleWhole, frame.consoleWholeReverse, frame.consoleWhole.length);
            frame.consoleString = frame.consoleWholeReverse.join('');
        }
        onStartDockingRobot: {
            var actionLabel=""
            var autoDockingLabel=""
            if (langue == "English") {
                actionLabel = "Start robot "
                autoDockingLabel = "auto docking process"
            } else {
                actionLabel = "机器人 "
                autoDockingLabel = "开始自动充电"
            }

            setMessageTop(2, actionLabel + "\"" + name + "\""+ autoDockingLabel);

            frame.startDockingRobot(ip)
            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + actionLabel + "\"" + name + "\"\n"+ autoDockingLabel + "\n");
            if (frame.consoleWhole.length === 20) {
                frame.consoleWhole.splice(0,1);
            }

            robotModel.reverse(frame.consoleWhole, frame.consoleWholeReverse, frame.consoleWhole.length);

            frame.consoleString = frame.consoleWholeReverse.join('');

        }
        onStopDockingRobot: {
            var actionLabel=""
            var autoDockingLabel=""
            if (langue == "English") {
                actionLabel = "Stop robot "
                autoDockingLabel = "auto docking process"
            } else {
                actionLabel = "停止机器人 "
                autoDockingLabel = "任务"
            }

            setMessageTop(2, actionLabel + "\"" + name + "\""+ autoDockingLabel);

            frame.stopDockingRobot(ip)
            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + actionLabel + "\"" + name + "\"\n"+ autoDockingLabel +"\n");
            if (frame.consoleWhole.length === 20) {
                frame.consoleWhole.splice(0,1);
            }

            robotModel.reverse(frame.consoleWhole, frame.consoleWholeReverse, frame.consoleWhole.length);

            frame.consoleString = frame.consoleWholeReverse.join('');
        }
        onInterruptDelay: frame.interruptDelay(ip)
        onStopPath: {
            var actionLabel=""
            var missionLabel=""
            if (langue == "English") {
                actionLabel = "Stop robot "
                missionLabel = "mission"
            } else {
                actionLabel = "停止机器人 "
                missionLabel = "任务"
            }

            setMessageTop(2, actionLabel + "\"" + name + "\""+ missionLabel);
            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + actionLabel + "\"" + name + "\"\n"+ missionLabel +"\n");

            if (frame.consoleWhole.length === 20) {
                frame.consoleWhole.splice(0,1);
            }

            robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
            frame.consoleString = frame.consoleWholeReverse.join('');
        }
        onPlayPath: {
            var actionLabel=""
            var missionLabel=""
            if (langue == "English") {
                actionLabel = "Pause robot "
                missionLabel = "mission"
            } else {
                actionLabel = "暂停机器人 "
                missionLabel = "路径"
            }
            setMessageTop(2, actionLabel + "\"" + name + "\"" + missionLabel);
            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + actionLabel + "\"" + name + "\"\n"+ missionLabel + "\n");

            if (frame.consoleWhole.length === 20) {
                frame.consoleWhole.splice(0,1);
            }

            robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
            frame.consoleString = frame.consoleWholeReverse.join('');
        }
        onPausePath: {
            var actionLabel=""
            var missionLabel=""
            if (langue == "English") {
                actionLabel = "Start robot "
                missionLabel = "mission"
            } else {
                actionLabel = "开始机器人 "
                missionLabel = "路径"
            }

            setMessageTop(2, actionLabel + "\"" + name + "\"" + missionLabel);
            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + actionLabel + "\"" + name + "\"\n"+ missionLabel +"\n");

            if (frame.consoleWhole.length === 20) {
                frame.consoleWhole.splice(0,1);
            }

            robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
            frame.consoleString = frame.consoleWholeReverse.join('');
        }
        onLoopPath: {
            var loopLabel=""
            if (langue == "English") {
                loopLabel = "Looping"
            } else {
                loopLabel = "开始循环"
            }

            setMessageTop(2, loopLabel);

            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + loopLabel +"\n");

            if (frame.consoleWhole.length === 20) {
                frame.consoleWhole.splice(0,1);
            }

            robotModel.reverse(consoleWhole, consoleWholeReverse, consoleWhole.length);
            frame.consoleString = frame.consoleWholeReverse.join('');
        }
        onUnloopPath: {
            var loopLabel=""
            if (langue == "English") {
                loopLabel = "Unlooping"
            } else {
                loopLabel = "停止循环"
            }

            setMessageTop(2, loopLabel);

            frame.consoleWhole.push(Qt.formatTime(new Date(),"hh:mm:ss") + ": " + loopLabel +"\n");

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
                    font.pointSize:Style.ubuntuSubHeadingSize
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
