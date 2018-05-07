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

    property variant consoleWhole: []
    property variant consoleWholeReverse: []
    property string consoleString

    property bool clickPausePlay
    property bool lastPointLoop
    property bool firstClick: false

    property int currentMenuIndex: 1

    signal pathSelected(string _pathName, string _groupName)
    signal startDockingRobot(string ip)
    signal stopDockingRobot(string ip)
    signal interruptDelay(string ip)

    /// signal for writing into console
    signal stopPath()
    signal playPath()
    signal pausePath()
    signal loopPath()
    signal unloopPath()

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
            width: 150

            background: Rectangle {
                color: "white"
                border.width: 1
                border.color: Style.lightGreyBorder
                radius: 3
            }

            anchors {
                verticalCenter: parent.verticalCenter
                left: parent.left
            }

            onClicked: pathListInPopup.open()

            PathListInPopup {
                id: pathListInPopup
                x: assignPath.width
                menuIndex: frame.currentMenuIndex
                pathModel: frame.pathModel
                onPathSelected: {
                    frame.pathSelected(pathName, groupName);
                    robotModel.newPathSignal(ip, _groupName, _pathName)

                }
            }
        }

        Button {
            id: stopPathButtonBis
            // prevents the icon from occasionally disappearing for no apparent reason

            height: parent.height - 2
            width: 28
            padding: 0

            background: Rectangle {
                color: "white"
                border.width: 1
                border.color: Style.lightGreyBorder
                radius: 3
            }

            anchors {
                verticalCenter: parent.verticalCenter
                left: assignPath.right
                leftMargin: 12
            }

            contentItem: Image {
                asynchronous: true
                source: "qrc:/icons/stop"
                fillMode: Image.PreserveAspectFit // For not stretching image
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }

            onClicked: {
                robotModel.stopPathSignal(ip)
                robotModel.stopButtonClicked = true;
                stopPath();
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
                left: stopPathButtonBis.right
                leftMargin: 12
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
            onClicked: {
                dockClicked()
            }
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
                        langue == "English" ? "隐藏地图上的机器人路径" : "Hide robot path on map"
                    } else {
                        langue == "English" ? "显示地图上的机器人路径" : "Show robot path on map"
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
                tooltip: pathIsOpen ? langue == "English" ? "隐藏路径" : " Hide robot path detail" : langue == "English" ? "显示路径" : " Show robot path detail"

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
                                    var waitTimeLabel = "";
                                    var humanActionLabel="";
                                    var SecondsLabel="";

//                                    waitTimeLabel = "toto"
                                    if (langue == "English") {
                                        waitTimeLabel = "等待 ："
                                        humanActionLabel="人为干预"
                                        SecondsLabel=" 秒"
                                    } else {
                                        waitTimeLabel = "Delay : "
                                        humanActionLabel="Human Action"
                                        SecondsLabel=" s"
                                    }

                                    if (playingPath === false) { // if robot not playingPath
                                        elapsedTimer.elapsed = 0;
                                        elapsedTimer2.elapsed = 0;
                                        customLabelWaitTime.humanActionClicked = false;
                                        customLabelWaitTime.previousStage = 0;
                                        if (waitTime === -1) {
                                            waitTimeText = humanActionLabel+"";
                                        } else {
                                            waitTimeText = waitTimeLabel+"" + waitTime +""+SecondsLabel;
                                        }
                                    } else {
                                        if (stage === index + 1) { /// if robot has reached point[index]
                                            if (waitTime === -1) { /// if human action case
                                                waitTimeText = humanActionLabel+"";
                                            } else if (waitTime >= 0) { /// if robot has a delay time before processing to next point
                                                elapsedTimer.restart();
                                                elapsedTime = waitTime - elapsedTimer.elapsed/1000;
                                                if (elapsedTime <= 0) { /// if delay time is finished
                                                   waitTimeText = waitTimeLabel+"" + waitTime + ""+SecondsLabel;
                                                } else { /// if delay time is still running
                                                    waitTimeText = waitTimeLabel+"" + elapsedTime + ""+SecondsLabel;
                                                }
                                            }
                                        } else if ((looping === true) && (customLabelWaitTime.previousStage === pathPoints.count - 1) && (stage === 0) && (index === customLabelWaitTime.previousStage)) { /// if last point
                                            if (waitTime === -1) { /// if human action case
                                                waitTimeText = humanActionLabel+"";
                                            } else if (waitTime >= 0) { /// if robot has a delay time before processing to next point
                                                elapsedTimer2.restart();
                                                elapsedTime2 = waitTime - elapsedTimer2.elapsed/1000;
                                                if (elapsedTime2 <= 0) { /// if delay time is finished
                                                    waitTimeText = waitTimeLabel+""+ waitTime + ""+SecondsLabel;

                                                } else { /// if delay time is still running
                                                    waitTimeText = waitTimeLabel+""+ elapsedTime2 + ""+SecondsLabel;
                                                }
                                            }
                                        } else { /// for point where robot is not while path is playing
                                            if (waitTime === -1) {
                                                waitTimeText = humanActionLabel+"";
                                            } else {
                                                waitTimeText = waitTimeLabel+"" + waitTime +""+SecondsLabel;
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
                                        if (stage === pathPoints.count) { //&& looping === true) {
                                            if (looping === true) {
                                                lastPointLoop = true;
                                            }
                                        }
                                    } else {
                                        if ((stage === 0) && (looping === false)) {
                                            clickPausePlay = false;
                                        }

                                        if ((stage === 1) && (looping === true)) { /// reset timer2
                                            elapsedTimer2.elapsed = 0;
                                        }

                                        /// stock previous stage of before last point while looping in a variable
                                        if ((stage === pathPoints.count - 1) && (looping === true)) {
                                            customLabelWaitTime.previousStage = pathPoints.count - 1;
                                        } /*else {

                                        }*/

                                        /// reset timer
                                        if ((stage === 0) && (looping === true)) { /// if we are between the last point and the first point, then reset the timer
                                            elapsedTimer.elapsed = 0;
                                            customLabelWaitTime.humanActionClicked = false;
                                        }

                                        if (stage === index + 1) { /// if robot has reached point[index]
                                            if (waitTime === -1) { /// if human action case
                                                if (frame.clickPausePlay === true) { /// case if we pressed the button pause
                                                    if (lastPointLoop === true) {
                                                        customLabelWaitTime.humanActionClicked = false;
                                                        lastPointLoop = false;
                                                    } else {
                                                        customLabelWaitTime.humanActionClicked = true;
                                                    }
                                                    frame.clickPausePlay = false;
                                                }

                                                if (customLabelWaitTime.humanActionClicked === false) {
                                                    color = Style.lightPurple;
                                                } else {
                                                    color = Style.lightGreyBackground;
                                                    lastPointLoop = false;
                                                }
                                            } else if (waitTime >= 0) { /// if robot has a delay time before processing to next point
                                                if (frame.clickPausePlay === true) { /// case if we pressed the button pause
                                                    if (lastPointLoop === true) {
                                                        elapsedTimer.elapsed = 0;
                                                        lastPointLoop = false;
                                                    } else {
                                                        elapsedTimer.elapsed = 1000000;
                                                    }
                                                    frame.clickPausePlay = false;
                                                }

                                                elapsedTimer.restart();
                                                elapsedTime = waitTime - elapsedTimer.elapsed/1000;
                                                if (elapsedTime <= 0) { /// if delay time is finished
                                                    color = Style.lightGreyBackground;
                                                    lastPointLoop = false;
                                                } else { /// if delay time is still running
                                                    color = Style.lightPurple;
                                                }
                                            }
                                        } else if ((looping === true) && (customLabelWaitTime.previousStage === pathPoints.count - 1) && (stage === 0) && (index === customLabelWaitTime.previousStage)) { /// if last point
                                            lastPointLoop = true;
                                            if (waitTime === -1) { /// if human action case

                                                if (customLabelWaitTime.humanActionClicked === false) {
                                                    color = Style.lightPurple;
                                                } else {
                                                    color = Style.lightGreyBackground;
                                                }
                                            } else if (waitTime >= 0) { /// if robot has a delay time before processing to next point
                                                if (robotModel.sendPointToRobot === true) {
                                                    if (lastPointLoop === true) {

                                                    } else {
                                                        elapsedTimer2.elapsed = 100000;
                                                    }


//                                                    elapsedTimer2.stop();
                                                }

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
//                                        console.log("Path is not playing, so do nothing");
                                    } else {
                                        if (stage === index + 1) { /// if robot has reached point[index]
                                            if (waitTime === -1) {
                                                if (humanActionClicked === false) { /// if human action case
//                                                    console.log("Case looping = false : Human Action clicked");
                                                    interruptDelay(ip);
                                                    /// now we need to make the button non clickable
                                                    customLabelWaitTime.humanActionClicked = true;
                                                } else {
//                                                    console.log("Human Action clicked already; do nothing");
                                                }
                                            } else if (waitTime >= 0) { /// if robot has a delay time before processing to next point
                                                elapsedTimer.restart();
                                                elapsedTime = waitTime - elapsedTimer.elapsed/1000;
                                                if (elapsedTime <= 0) { /// if delay time is finished
//                                                    console.log("Case looping = false : time elapsed finished, do nothing");
                                                } else { /// if delay time is still running
                                                    interruptDelay(ip);
                                                    /// now we need to interrupt the time running / make the button non clickable
                                                    elapsedTimer.elapsed = 1000000; /// we set elapsedTimer to be very big, so that (waitTime - elapsedTimer.elapsed/1000) < 0
                                                }
                                            }
                                        } else if ((looping === true) && (customLabelWaitTime.previousStage === pathPoints.count - 1) && (stage === 0) && (index === customLabelWaitTime.previousStage)) {
                                            if (waitTime === -1) {
                                                if (humanActionClicked === false) { /// if human action case
//                                                    console.log("Case looping = false : Human Action clicked");
                                                    interruptDelay(ip);
                                                    /// now we need to make the button non clickable
                                                    customLabelWaitTime.humanActionClicked = true;
                                                } else {
//                                                    console.log("Human Action clicked already; do nothing");
                                                }
                                            } else if (waitTime >= 0) { /// if robot has a delay time before processing to next point
                                                elapsedTimer2.restart();
                                                elapsedTime2 = waitTime - elapsedTimer2.elapsed/1000;
                                                if (elapsedTime2 <= 0) { /// if delay time is finished
//                                                    console.log("Case looping = false : time elapsed finished, do nothing");
                                                } else { /// if delay time is still running
                                                    interruptDelay(ip);
                                                    /// now we need to interrupt the time running / make the button non clickable
                                                    elapsedTimer2.elapsed = 1000000; /// we set elapsedTimer to be very big, so that (waitTime - elapsedTimer.elapsed/1000) < 0
                                                }
                                            }
                                        } else { /// for point where robot is not
//                                            console.log("Robot is not here, do nothing");
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
                    if (playingPath === true) {
                        robotModel.pausePathSignal(ip);
                    } else {
                        if (firstClick === false) {
                            robotModel.playPathSignal(ip);
                            firstClick = true;
                        } else {
                            clickPausePlay = true;
                            robotModel.playPathSignal(ip);
                        }
                    }

                    if (playingPath) {
                        playPath();
                    } else {
                        pausePath();
                    }

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
                    if (goHomeButton.dockButtonClicked === true) {
                        robotModel.stopPathSignal(ip);
                        clickPausePlay = true;
                        goHomeButton.dockButtonClicked = false;
                    } else {
                        robotModel.stopPathSignal(ip);
                        firstClick = false;
                        clickPausePlay = false;
                        lastPointLoop = false;
                    }
                    robotModel.stopButtonClicked = true;
                    stopPath();
                }
            }

            SmallButton {
                id: loopPathButton
                // prevents the icon from occasionally disappearing for no apparent reason
                backColor: "white"
                height: parent.height - 2
                width: playPausePathButton.width
                imgSrc: "qrc:/icons/reset"
//                tooltip: langue == "English" ? "在路径上循环" : "Loop the path"
                tooltip: looping ? langue == "English" ?"停止循环":"Unloop": langue == "English" ?"开始循环":"Loop"
                checkable: true
                checked: looping

                anchors {
                    verticalCenter: parent.verticalCenter
                    right: goHomeButton.left
                    rightMargin: (bottomItem.width - playPausePathButton.anchors.leftMargin * 2 - playPausePathButton.width * 4) / 3
                }

                onClicked: {
                    robotModel.setLoopingPathSignal(ip, !looping);

                    if (looping) {
                        unloopPath();
                    } else {
                        loopPath();
                    }
                }
            }

            property bool homeButtonClicked: false

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
//                tooltip: langue == "English" ? "让机器人去充电桩" : "Send the robot to its docking station"
                tooltip: (dockStatus === 3) ? langue == "English" ?"停止自动对接":"Stop auto docking" : langue == "English" ?"自动对接":"Auto docking"

                anchors {
                    verticalCenter: parent.verticalCenter
                    right: parent.right
                    rightMargin: 12
                }

                enabled: dockStatus != -2
                property bool dockButtonClicked: false
                onClicked: {
                    dockClicked()
                    dockButtonClicked = true;
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
