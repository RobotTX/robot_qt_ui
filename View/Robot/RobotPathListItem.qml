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
    property string homeXRobot
    property string homeYRobot

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
            text: langue == "English" ? "Assign Path" : "设置路径"
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
                    robotModel.newPathSignal(ip, groupName, pathName);
                    pathListInPopup.close();
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
                console.log("homeXRobot = " + homeXRobot + "homeYRobot = " + homeYRobot + " homeX = " + homeX + " homeY = " + homeY);
                if (homeXRobot === "-150" && homeYRobot === "-150" && homeX === -1 && homeY === -1) {
                    noHomeDialog.open();
                } else {
                    dockClicked()
                }
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
                        langue == "English" ? "Hide robot path on map" : "隐藏地图上的机器人路径"
                    } else {
                        langue == "English" ? "Show robot path on map" : "显示地图上的机器人路径"
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
                font.pointSize: Style.ubuntuHeading2Size
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
                tooltip: pathIsOpen ? langue == "English" ? "Hide robot path detail" : " 隐藏路径细节" : langue == "English" ? "Show robot path detail" : " 查看路径细节"

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
                                font.pointSize: Style.ubuntuTextSize
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
                                    font.pointSize: Style.ubuntuTextSize
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                }

                                background: Rectangle {
                                    color: customLabelWaitTime.setColor()
                                }

                                height: 20
                                font.pointSize: Style.ubuntuTitleSize
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
                                        waitTimeLabel = "Delay : "
                                        humanActionLabel="Human Action"
                                        SecondsLabel=" s"
                                    } else {
                                        waitTimeLabel = "等待 ："
                                        humanActionLabel="人为干预"
                                        SecondsLabel=" 秒"
                                    }

                                    if (playingPath === false) { // if robot not playingPath
//                                        console.log("playingPath false");
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
//                                        console.log("playingPath true");
//                                        elapsedTimer.elapsed = 0;
                                        if (stage === index + 1) { /// if robot has reached point[index]
                                            if (waitTime === -1) { /// if human action case
                                                waitTimeText = humanActionLabel+"";
                                            } else if (waitTime >= 0) { /// if robot has a delay time before processing to next point
//                                                console.log("elapsedTimer.elapsed = " + elapsedTimer.elapsed);
//                                                elapsedTimer.restart();
                                                elapsedTimer.start();
                                                elapsedTime = waitTime - elapsedTimer.elapsed/1000;
//                                                console.log("elapsedTime = " + elapsedTime + " -- waitTime = " + waitTime + " -- elapsedTimer.elapsed/1000 = " + elapsedTimer.elapsed/1000);
                                                if (elapsedTime <= 0) { /// if delay time is finished
                                                   waitTimeText = waitTimeLabel+"" + waitTime + ""+SecondsLabel;
//                                                   console.log("delay time is finished");
                                                } else { /// if delay time is still running
//                                                    console.log("delay time is not finished");
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
//                                        console.log("playingPath = false");
                                        color = Style.lightGreyBackground;
                                        if (stage === pathPoints.count) { //&& looping === true) {
                                            if (looping === true) {
                                                lastPointLoop = true;
//                                                console.log("lastpointloop = true");
                                            }
                                        }
                                    } else {
//                                        console.log("playingPath = true");
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
//                                            console.log("we reset timer");
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
//                                                        console.log("laspointloop = true");
                                                        elapsedTimer.elapsed = 0;
                                                        lastPointLoop = false;
                                                    } else {
                                                        if (robotModel.assignPathClicked === true) {
                                                            elapsedTimer.elapsed = 0;
//                                                            console.log("assign click");
                                                            robotModel.assignPathClicked = false;
                                                        } else {
                                                            elapsedTimer.elapsed = 1000000;
//                                                            console.log("we set elapsedTime1 to 1000000 lastpointloop == false");
                                                        }


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
                visible: robotMode === 0 ? true : false
                width: 32
                imgSrc: playingPath ? "qrc:/icons/pause" : "qrc:/icons/play"
                tooltip: { if(playingPath) {
                            langue == "English" ? "Puase robot path" : "暂停机器人路径"
                    } else {
                        langue == "English" ? "Play robot path" : "开始机器人路径"
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
                visible: robotMode === 0 ? true : false
                width: playPausePathButton.width
                imgSrc: "qrc:/icons/stop"
                tooltip: langue == "English" ? "Stop robot" : "停止机器人"

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
                visible: robotMode === 0 ? true : false
                width: playPausePathButton.width
                imgSrc: "qrc:/icons/reset"
//                tooltip: langue == "English" ? "在路径上循环" : "Loop the path"
                tooltip: looping ? langue == "English" ?"Unloop":"停止循环": langue == "English" ?"Loop":"循环"
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
                visible: robotMode === 0 ? true : false
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
//                tooltip: langue == "English" ? "让机器人去充电站" : "Send the robot to its docking station"
//                tooltip: (dockStatus === 3) ? langue == "English" ?"停止自动充电":"Stop auto docking" : langue == "English" ?"自动充电":"Auto docking"
                tooltip: {
                    if (langue === "English") {
                        if (dockStatus === 3) {
                            "Stop auto docking" ;
                        } else if (dockStatus === 1) {
                            "Charging";
                        } else {
                            "Auto docking";
                        }
                    } else {
                        if (dockStatus === 3) {
                            "停止自动充电";
                        } else if (dockStatus === 1) {
                            "充电中";
                        } else {
                            "自动充电";
                        }
                    }
                }

                anchors {
                    verticalCenter: parent.verticalCenter
                    right: parent.right
                    rightMargin: 12
                }

                enabled: dockStatus != -2
                property bool dockButtonClicked: false
                onClicked: {
                    console.log("homeXRobot = " + homeXRobot + " homeYRobot = " + homeYRobot + " homeX = " + homeX + " homeY = " + homeY);
                    if (homeXRobot === "-150" && homeYRobot === "-150" && homeX === -1 && homeY === -1) {
                        noHomeDialog.open();
                    } else {
                        dockClicked()
                        dockButtonClicked = true;
                    }
                }
            }

            CustomDialog {
                id: noHomeDialog
                parent: ApplicationWindow.overlay
                x: (parent.width - width) / 2
                y: (parent.height - height) / 2
                height: 130
                topMarginLabel: langue === "English" ? 10 : 20;
                leftMarginLabel: langue === "English" ? 40 : 80;
                title: langue == "English" ? "WARNING" : "警告"
                message: langue == "English" ? "Please assign charging station to " + name + " before auto docking." : "自动充电前，请分配充电站给" + name
                acceptMessage: langue == "English" ? "OK" : "确认"
            }

            Image {
                id: manualControlImage
                asynchronous: true
                visible: robotMode === 0 ? false : true
                source: "qrc:/icons/manual_control"
                fillMode: Image.Pad
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.fill: parent
            }
        }
    }

    function dockClicked(){
        if(dockStatus != -2){
            if(dockStatus == 3){
                frame.stopDockingRobot(ip);
            } else {
                frame.startDockingRobot(ip);
            }
        }
    }
}
