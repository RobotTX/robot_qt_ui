import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Model/Robot"
import "../Custom"

Frame {
    id: scanMapListItemFrame

    property bool selected

    signal stopScanning()
    signal playPauseScanning(bool scanning)
    signal playPauseExploring(bool exploring)
    signal sendTeleop(int index)
    signal select()
    signal centerOnRobot(string ip)

    padding: 0
    height: 260

    background: Rectangle {
        color: Style.lightGreyBackground
    }

    Rectangle {

        anchors.fill: parent
        anchors.margins: 15
        color: selected ? Style.selectedItemColor : hovered ? Style.lightGreyBackgroundHover : Style.lightGreyBackground
        radius: 8

        Timer {
            id: timer
            interval: 50
            triggeredOnStart: false
            repeat: true
            property int elapsed_time
            onTriggered: elapsed_time += interval;
        }

        MouseArea {
            id: mouseArea
            property int delay: 300
            anchors.fill: parent
            enabled: connected && !busy
            onClicked: {
                if(timer.elapsed_time === 0 || timer.elapsed_time > 300){
                    if(timer.elapsed_time === 0)
                        timer.start();
                    console.log("select event called");
                    scanMapListItemFrame.select();
                    if(timer.elapsed_time > 300)
                        timer.elapsed_time = 0
                } else {
                    console.log("reset time elapse");
                    timer.elapsed_time = 0;
                }
            }
            onDoubleClicked: {
                if(!selected)
                    scanMapListItemFrame.select();
                console.log("double clicked ");
                scanMapListItemFrame.centerOnRobot(ip);
            }
        }

        Image {
            id: validIcon
            source: mapReceived ? "qrc:/icons/valid" : "qrc:/icons/notValid"

            CustomToolTip {
                visible: !mapReceived && hovered
                text: mapReceived ? "" : "The map has not been received yet"
            }

            fillMode: Image.Pad
            anchors {
                top: parent.top
                left: parent.left
                topMargin: 5
                leftMargin: 5
            }
        }

        CustomLabel {
            id: nameLabel
            text: qsTr(name)
            anchors {
                verticalCenter: validIcon.verticalCenter
                left: validIcon.right
                right: closeBtn.left
            }
        }

        Button {
            id: closeBtn
            anchors {
                verticalCenter: validIcon.verticalCenter
                right: parent.right
                rightMargin: 5
            }

            background: Rectangle {
                color: parent.hovered ? Style.lightGreyBackgroundHover : "transparent"
                radius: parent.hovered ? Style.smallBtnWidth / 2 : 0
            }

            width: Style.smallBtnWidth
            height: Style.smallBtnHeight
            padding: 0

            contentItem: Image {
                asynchronous: true
                source: "qrc:/icons/closeBtn"
                fillMode: Image.Pad // For not stretching image
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }
            onClicked: stopScanning()
        }

        CancelButton {
            id: playScanBtn
            txt: scanning ? "Pause" : "Start Scanning"
            enabled: connected
            anchors {
                top: nameLabel.bottom
                left: parent.left
                right: parent.right
                topMargin: 10
                leftMargin: 5
                rightMargin: 5
            }
            onClicked: playPauseScanning(scanning)
        }

        CancelButton {
            id: autoScanBtn
            txt: onAutomatic ? "Stop Exploring" : "Start Exploring"
            enabled: connected
            anchors {
                top: playScanBtn.bottom
                left: parent.left
                right: parent.right
                topMargin: 10
                leftMargin: 5
                rightMargin: 5
            }
            onClicked: playPauseExploring(onAutomatic)
        }

        Teleop {
            id: teleop
            enabled: connected
            anchors {
                top: autoScanBtn.bottom
                topMargin: 20
                horizontalCenter: parent.horizontalCenter
            }
            onSendTeleop: scanMapListItemFrame.sendTeleop(index)
        }

        ToolSeparator {
            orientation: Qt.Horizontal
            anchors {
                top: teleop.bottom
                left: parent.left
                right: parent.right
                topMargin: 20
            }
        }
    }

    Rectangle {
        id: connectedRect
        visible: !connected || busy
        radius: 3
        color: Style.lightGreyBackground
        opacity: 0.5
        anchors.fill: parent
        //anchors.topMargin: 40
    }

    Image {
        id: connectedImage
        visible: !connected
        source: "qrc:/icons/disco"
        fillMode: Image.PreserveAspectFit
        width: 80
        anchors {
            top: parent.top
            topMargin: 52
            horizontalCenter: connectedRect.horizontalCenter
        }
    }

    Label {
        visible: !connected
        text: "This robot is disconnected"
        color: "#ec6262"
        anchors {
            horizontalCenter: connectedImage.horizontalCenter
            top: connectedImage.bottom
        }
        wrapMode: Text.WordWrap
        width: connectedImage.width
        horizontalAlignment: Text.AlignHCenter
    }

    CustomBusyIndicator {
        running: busy
        anchors.fill: parent
    }
}
