import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Model/Path"
import "../../Model/Robot"
import "../Custom"
import "../Path"

Frame {
    id: frame
    property Robots robotModel
    property Paths pathModel
    property string langue

    signal pathSelected(string _pathName, string _groupName)
    signal startDockingRobot(string ip)
    signal stopDockingRobot(string ip)
    signal interruptDelay(string ip)

    height: noPathItem.visible ? noPathItem.height : pathItem.height
    padding: 0

    background: Rectangle {
        color: Style.lightGreyBackground
    }

    Item {
        id: noPathItem
        visible: pathName === ""
        height: 23

        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
        }

        Button {
            id: assignPath
            text: langue == "English" ? "设置路径" : "Assign Path"
            height: parent.height - 2
            padding: 0

            background: Rectangle {
                color: "white"
                border.width: 1
                border.color: Style.lightGreyBorder
                radius: 3
            }

            anchors {
                verticalCenter: parent.verticalCenter
                left: parent.left
                right: homeButton.left
                rightMargin: 12
            }

            onClicked: pathListInPopup.open()

            PathListInPopup {
                id: pathListInPopup
                x: assignPath.width
                onVisibleChanged: if(!visible) currentMenuIndex = -1
                pathModel: frame.pathModel
                onPathSelected: {
                    frame.pathSelected(pathName, groupName);
                    currentMenuIndex = -1;
                    close();
                }
            }
        }

        Button {
            id: homeButton

            height: parent.height - 2
            width: 32
            padding: 0

            background: Rectangle {
                color: "white"
                border.width: 1
                border.color: Style.lightGreyBorder
                radius: 3
            }

            anchors {
                verticalCenter: parent.verticalCenter
                right: parent.right
            }

            contentItem: Image {
                asynchronous: true
                source: {
                    switch(dockStatus){
                        case -4:
                            "qrc:/icons/home_red2_fill"
                        break;
                        case -3:
                            "qrc:/icons/home_red2_fill"
                        break;
                        case -2:
                            "qrc:/icons/noHome"
                        break;
                        case -1:
                            "qrc:/icons/home_red2_fill"
                        break;
                        case 0:
                            "qrc:/icons/home"
                        break;
                        case 1:
                            "qrc:/icons/home_blue_fill"
                        break;
                        case 2:
                            "qrc:/icons/home_yellow"
                        break;
                        case 3:
                            "qrc:/icons/home_yellow_fill"
                        break;
                        default:
                            "qrc:/icons/notValid"
                        break;
                    }
                }
                fillMode: Image.PreserveAspectFit // For not stretching image
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }
            onClicked: dockClicked()
        }
    }

    Rectangle {
        id: pathItem
        visible: !noPathItem.visible
        height: topItem.height + bottomItem.height + (pathIsOpen ? Math.min(89, flick.contentHeight) : 0)

        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
        }

        color: "white"
        border.width: 1
        border.color: Style.lightGreyBorder // up and down
        radius: 3

        Item {
            id: topItem
            height: 28

            anchors {
                top: parent.top
                left: parent.left
                right: parent.right
            }

            SmallButton {
                id: hideShowPathButton
                // prevents the icon from occasionally disappearing for no apparent reason
                backColor: "white"
                height: parent.height - 2
                width: 32
                imgSrc: pathIsVisible ? "qrc:/icons/visible" : "qrc:/icons/invisible"
                tooltip: {
                    if (pathIsVisible) {
                        langue == "English" ? "隐藏地图上的机器人路径" : "Hide the path of the robot on the map"
                    } else {
                        langue == "English" ? "显示地图上的机器人路径" : "Show the path of the robot on the map"
                    }
                }

                anchors {
                    verticalCenter: parent.verticalCenter
                    left: parent.left
                    leftMargin: 8
                }

                onClicked: {
                    robotModel.hideShowPathOnMap(ip);
                    pathModel.visiblePathChanged();
                }
            }

            CustomLabel {
                text: qsTr(pathName)
                height: parent.height
                color: Style.blackMenuTextColor

                verticalAlignment: Text.AlignVCenter

                anchors {
                    verticalCenter: parent.verticalCenter
                    left: hideShowPathButton.right
                    right: expandPathButton.left
                    leftMargin: 5
                    rightMargin: 5
                }
            }

            SmallButton {
                id: expandPathButton
                // prevents the icon from occasionally disappearing for no apparent reason
                backColor: "white"
                height: parent.height - 2
                width: 32
                imgSrc: pathIsOpen ? "qrc:/icons/fold" : "qrc:/icons/unfold"

                anchors {
                    verticalCenter: parent.verticalCenter
                    right: parent.right
                    rightMargin: 8
                }

                onClicked: robotModel.openPath(ip)
            }

            // down line for 1st rectangle
            Rectangle {
                id: borderBottom
                height: 1
                width: parent.width
                anchors.bottom: parent.bottom
                color: Style.lightGreyBorder
            }
        }

        Item {
            visible: pathIsOpen
            anchors {
                top: topItem.bottom
                bottom: bottomItem.top
                left: parent.left
                right: parent.right
            }

            Flickable {
                id: flick
                ScrollBar.vertical: ScrollBar { }
                contentHeight: contentItem.childrenRect.height
                anchors.fill: parent
                clip: true



                Rectangle {
                    anchors.fill: parent
                    color: Style.lightGreyBackground
                }

                Column {
                    /// The list containing both the graphical and model of the robots in the menu
                    Repeater {
                        model: pathPoints
                        delegate: delegatePathPoint
                    }

                    Component {
                        id: delegatePathPoint
                        Frame {
                            height: 25
                            width: flick.width
                            padding: 0
                            background: Rectangle {
                                anchors.fill: parent
                                color: Style.lightGreyBackground
                            }

                            Rectangle {
                                height: 15
                                width: 2
                                color: {
                                    if(stage >= 0){
                                        if(stage >= index)
                                            Style.darkSkyBlue
                                        else
                                            "#d7d7d7"
                                    } else {
                                        if(Math.abs(stage + 1) >= index)
                                            Style.errorColor2
                                        else
                                            "#d7d7d7"
                                    }
                                }
                                anchors.horizontalCenter: rect.horizontalCenter
                                anchors.bottom: rect.top
                                visible: index > 0
                            }

                            Rectangle {
                                id: rect
                                height: 10
                                width: height
                                radius: height
                                color: {
                                    if(stage >= 0){
                                        if(stage > index)
                                            Style.darkSkyBlue
                                        else
                                            "#d7d7d7"
                                    } else {
                                        if(Math.abs(stage + 1) > index)
                                            Style.errorColor2
                                        else
                                            "#d7d7d7"
                                    }
                                }
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.left: parent.left
                                anchors.leftMargin: 25
                            }

                            /// The item displaying the name of the pathpoint
                            CustomLabel {
                                id: customLabelPathPointName
                                text: qsTr(pathPointName)
                                font.pixelSize: 14
                                color: Style.midGrey2
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.left: rect.right
                                anchors.right: customLabelWaitTime.left
                                anchors.leftMargin: 10
                                anchors.rightMargin: 5
                            }

                            Timer  {
                                    id: elapsedTimer
                                    triggeredOnStart: false
                                    interval: 1000;
                                    property int elapsed: 0
                                    onTriggered: {
                                        elapsed += interval;
                                    }
                                }

                            Timer  {
                                    id: elapsedTimer2
                                    triggeredOnStart: false
                                    interval: 1000;
                                    property int elapsed: 0
                                    onTriggered: {
                                        elapsed += interval;
                                    }
                                }

                            Button {
                                id: customLabelWaitTime
                                text: textButton.text
                                contentItem: Text {
                                    id: textButton
                                    text: customLabelWaitTime.setWaitTimeText()
                                    color: Style.midGrey2
                                    font.pointSize: 10
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                }

                                background: Rectangle {
                                    color: customLabelWaitTime.setColor()
                                }

                                height: 20
                                font.pixelSize: 14
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.right: parent.right
                                anchors.rightMargin: 5
                                onClicked: customLabelWaitTime.setInterruptionDelay()

                                property bool humanActionClicked: false
                                property int previousStage: 0


                                function setWaitTimeText() {
                                    var waitTimeText = "";
                                    var elapsedTime = "";
                                    var elapsedTime2 = "";
                                    if (playingPath === false) { // if robot not playingPath
                                        elapsedTimer.elapsed = 0;
                                        elapsedTimer2.elapsed = 0;
                                        customLabelWaitTime.humanActionClicked = false;
                                        customLabelWaitTime.previousStage = 0;
                                        if (waitTime === -1) {
                                            waitTimeText = "Human Action";
                                        } else {
                                            waitTimeText = "Delay : " + waitTime + " s";
                                        }
                                    } else {
                                        /// reset timer
                                        if ((stage === 0) && (looping === true)) { /// if we are between the last point and the first point, then reset the timer
                                            elapsedTimer.elapsed = 0;
                                            customLabelWaitTime.humanActionClicked = false;
                                        }

                                        if ((stage === 1) && (looping === true)) { /// reset timer2
                                            elapsedTimer2.elapsed = 0;
                                        }

                                        /// stock previous stage of before last point while looping in a variable
                                        if ((stage === pathPoints.count - 1) && (looping === true)) {
                                            customLabelWaitTime.previousStage = pathPoints.count - 1;
                                        } else {

                                        }

                                        if (stage === index + 1) { /// if robot has reached point[index]
                                            if (waitTime === -1) { /// if human action case
                                                waitTimeText = "Human Action";
                                            } else if (waitTime >= 0) { /// if robot has a delay time before processing to next point
                                                elapsedTimer.restart();
                                                elapsedTime = waitTime - elapsedTimer.elapsed/1000;
                                                if (elapsedTime <= 0) { /// if delay time is finished
                                                    waitTimeText = "Delay : " + waitTime + " s";

                                                } else { /// if delay time is still running
                                                    waitTimeText = "Delay : " + elapsedTime + " s";
                                                }
                                            }
                                        } else if ((looping === true) && (customLabelWaitTime.previousStage === pathPoints.count - 1) && (stage === 0) && (index === customLabelWaitTime.previousStage)) { /// if last point
                                            if (waitTime === -1) { /// if human action case
                                                waitTimeText = "Human Action";
                                            } else if (waitTime >= 0) { /// if robot has a delay time before processing to next point
                                                elapsedTimer2.restart();
                                                elapsedTime2 = waitTime - elapsedTimer2.elapsed/1000;
                                                if (elapsedTime2 <= 0) { /// if delay time is finished
                                                    waitTimeText = "Delay : " + waitTime + " s";

                                                } else { /// if delay time is still running
                                                    waitTimeText = "Delay : " + elapsedTime2 + " s";
                                                }
                                            }
                                        } else { /// for point where robot is not
                                            if (waitTime === -1) {
                                                waitTimeText = "Human Action";
                                            } else {
                                                waitTimeText = "Delay : " + waitTime + " s";
                                            }
                                        }
                                    }
                                    return waitTimeText;
                                }

                                function setColor() {
                                    var color = "";
                                    var elapsedTime = "";
                                    var elapsedTime2 = "";
                                    if (playingPath === false) { // if robot not playingPath
                                        color = Style.lightGreyBackground;
                                    } else {
                                        if (stage === index + 1) { /// if robot has reached point[index]
                                            if (waitTime === -1) { /// if human action case
                                                if (customLabelWaitTime.humanActionClicked === false) {
                                                    color = Style.lightPurple;
                                                } else {
                                                    color = Style.lightGreyBackground;
                                                }
                                            } else if (waitTime >= 0) { /// if robot has a delay time before processing to next point
                                                elapsedTimer.restart();
                                                elapsedTime = waitTime - elapsedTimer.elapsed/1000;
                                                if (elapsedTime <= 0) { /// if delay time is finished
                                                    color = Style.lightGreyBackground;
                                                } else { /// if delay time is still running
                                                    color = Style.lightPurple;
                                                }
                                            }
                                        } else if ((looping === true) && (customLabelWaitTime.previousStage === pathPoints.count - 1) && (stage === 0) && (index === customLabelWaitTime.previousStage)) {
                                            if (waitTime === -1) { /// if human action case
                                                if (customLabelWaitTime.humanActionClicked === false) {
                                                    color = Style.lightPurple;
                                                } else {
                                                    color = Style.lightGreyBackground;
                                                }
                                            } else if (waitTime >= 0) { /// if robot has a delay time before processing to next point
                                                elapsedTimer2.restart();
                                                elapsedTime2 = waitTime - elapsedTimer2.elapsed/1000;
                                                if (elapsedTime2 <= 0) { /// if delay time is finished
                                                    color = Style.lightGreyBackground;
                                                } else { /// if delay time is still running
                                                    color = Style.lightPurple;
                                                }
                                            }
                                        } else { /// for point where robot is not
                                            color = Style.lightGreyBackground;
                                        }
                                    }
                                    return color;
                                }

                                function setInterruptionDelay() {
                                    var elapsedTime = "";
                                    var elapsedTime2 = "";
                                    if (playingPath === false) { // if robot not playingPath
                                        console.log("Path is not playing, so do nothing");
                                    } else {
                                        if (stage === index + 1) { /// if robot has reached point[index]
                                            if (waitTime === -1) {
                                                if (humanActionClicked === false) { /// if human action case
                                                    console.log("Case looping = false : Human Action clicked");
                                                    interruptDelay(ip);
                                                    /// now we need to make the button non clickable
                                                    customLabelWaitTime.humanActionClicked = true;
                                                } else {
                                                    console.log("Human Action clicked already; do nothing");
                                                }
                                            } else if (waitTime >= 0) { /// if robot has a delay time before processing to next point
                                                elapsedTimer.restart();
                                                elapsedTime = waitTime - elapsedTimer.elapsed/1000;
                                                if (elapsedTime <= 0) { /// if delay time is finished
                                                    console.log("Case looping = false : time elapsed finished, do nothing");
                                                } else { /// if delay time is still running
                                                    interruptDelay(ip);
                                                    /// now we need to interrupt the time running / make the button non clickable
                                                    elapsedTimer.elapsed = 1000000; /// we set elapsedTimer to be very big, so that (waitTime - elapsedTimer.elapsed/1000) < 0
                                                }
                                            }
                                        } else if ((looping === true) && (customLabelWaitTime.previousStage === pathPoints.count - 1) && (stage === 0) && (index === customLabelWaitTime.previousStage)) {
                                            if (waitTime === -1) {
                                                if (humanActionClicked === false) { /// if human action case
                                                    console.log("Case looping = false : Human Action clicked");
                                                    interruptDelay(ip);
                                                    /// now we need to make the button non clickable
                                                    customLabelWaitTime.humanActionClicked = true;
                                                } else {
                                                    console.log("Human Action clicked already; do nothing");
                                                }
                                            } else if (waitTime >= 0) { /// if robot has a delay time before processing to next point
                                                elapsedTimer2.restart();
                                                elapsedTime2 = waitTime - elapsedTimer2.elapsed/1000;
                                                if (elapsedTime2 <= 0) { /// if delay time is finished
                                                    console.log("Case looping = false : time elapsed finished, do nothing");
                                                } else { /// if delay time is still running
                                                    interruptDelay(ip);
                                                    /// now we need to interrupt the time running / make the button non clickable
                                                    elapsedTimer2.elapsed = 1000000; /// we set elapsedTimer to be very big, so that (waitTime - elapsedTimer.elapsed/1000) < 0
                                                }
                                            }
                                        } else { /// for point where robot is not
                                            console.log("Robot is not here, do nothing");
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            Rectangle {
                height: 1
                width: parent.width
                anchors.bottom: parent.bottom
                color: Style.lightGreyBorder
            }
        }

        Item {
            id: bottomItem

            height: 28

            anchors {
                bottom: parent.bottom
                left: parent.left
                right: parent.right
            }

            SmallButton {
                id: playPausePathButton
                // prevents the icon from occasionally disappearing for no apparent reason
                backColor: "white"
                height: parent.height - 2
                width: 32
                imgSrc: playingPath ? "qrc:/icons/pause" : "qrc:/icons/play"
                tooltip: { if(playingPath) {
                            langue == "English" ? "暂停机器人的任务" : "Pause the robot in its path"
                    } else {
                        langue == "English" ? "继续机器人的任务" : "Play the path of the robot"
                    }
                }


                anchors {
                    verticalCenter: parent.verticalCenter
                    left: parent.left
                    leftMargin: 12
                }

                onClicked: {
                    playingPath ? robotModel.pausePathSignal(ip): robotModel.playPathSignal(ip);
                }

            }
            SmallButton {
                id: stopPathButton
                // prevents the icon from occasionally disappearing for no apparent reason
                backColor: "white"
                height: parent.height - 2
                width: playPausePathButton.width
                imgSrc: "qrc:/icons/stop"
                tooltip: langue == "English" ? "停止机器人的任务" : "Stop the robot in its path"

                anchors {
                    verticalCenter: parent.verticalCenter
                    left: playPausePathButton.right
                    leftMargin: (bottomItem.width - playPausePathButton.anchors.leftMargin * 2 - playPausePathButton.width * 4) / 3
                }

                onClicked: {
                    robotModel.stopPathSignal(ip);
                }
            }

            SmallButton {
                id: loopPathButton
                // prevents the icon from occasionally disappearing for no apparent reason
                backColor: "white"
                height: parent.height - 2
                width: playPausePathButton.width
                imgSrc: "qrc:/icons/reset"
                tooltip: langue == "English" ? "在路径上循环" : "Loop the path"
                checkable: true
                checked: looping

                anchors {
                    verticalCenter: parent.verticalCenter
                    right: goHomeButton.left
                    rightMargin: (bottomItem.width - playPausePathButton.anchors.leftMargin * 2 - playPausePathButton.width * 4) / 3
                }

                onClicked: {
                    robotModel.setLoopingPathSignal(ip, !looping);
                }
            }

            SmallButton {
                id: goHomeButton
                // prevents the icon from occasionally disappearing for no apparent reason
                backColor: "white"
                height: parent.height - 2
                width: playPausePathButton.width
                padding: 0
                imgSrc: {
                    switch(dockStatus){
                        case -4:
                            "qrc:/icons/home_red2_fill"
                        break;
                        case -3:
                            "qrc:/icons/home_red2_fill"
                        break;
                        case -2:
                            "qrc:/icons/noHome"
                        break;
                        case -1:
                            "qrc:/icons/home_red2_fill"
                        break;
                        case 0:
                            "qrc:/icons/home"
                        break;
                        case 1:
                            "qrc:/icons/home_blue_fill"
                        break;
                        case 2:
                            "qrc:/icons/home_yellow"
                        break;
                        case 3:
                            "qrc:/icons/home_yellow_fill"
                        break;
                        default:
                            "qrc:/icons/notValid"
                        break;
                    }
                }
                tooltip: langue == "English" ? "让机器人去充电桩" : "Send the robot to its docking station"

                anchors {
                    verticalCenter: parent.verticalCenter
                    right: parent.right
                    rightMargin: 12
                }

                enabled: dockStatus != -2

                onClicked: {
                    dockClicked()
                    playingPath = false;
                }
            }
        }

        // console displaying actions
        Rectangle {
            id: idConsole
            visible: true
            height: 70
            anchors {
                top: pathItem.bottom
                topMargin: 5
                left: parent.left
                right: parent.right
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
                        text: robotModel.msg

                        font.pixelSize: 14
                        color: Style.midGrey2
//                        {
//                            if (robotModel.statusColor === -1)
//                                Style.midGrey2
//                            else if (robotModel.statusColor === 0)
//                                Style.errorColor
//                            else if (robotModel.statusColor === 1)
//                                Style.warningColor
//                            else if (robotModel.statusColor === 2)
//                                Style.successColor
//                            else if (robotModel.statusColor === 3)
//                                Style.infoColor
//                        }
                    }
                }

            }
        }
    }

    function dockClicked(){
        if(dockStatus != -2){
            if(dockStatus == 3){
                frame.stopDockingRobot(ip);
                console.log("Stop docking");
            } else {
                frame.startDockingRobot(ip);
                console.log("Start docking");
            }
        }
    }
}
