import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Model/Robot"
import "../Custom"

Frame {

    id: scanMapListItemFrame

    signal stopScanning(string ip)
    signal playPauseScanning(string ip, bool scanning)
    signal sendTeleop(string ip, int index)

    padding: 0
    enabled: !busy
    height: 220

    background: Rectangle {
        color: "transparent"
    }

    Rectangle {
        anchors.fill: parent
        anchors.margins: 15
        color: "transparent"

        Image {
            id: validIcon
            source: mapReceived ? "qrc:/icons/valid" : "qrc:/icons/notValid"
            fillMode: Image.Pad
            anchors {
                top: parent.top
                left: parent.left
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
            onClicked: stopScanning(ip)
        }

        CancelButton {
            id: playScanBtn
            txt: scanning ? "Pause" : "Start Scanning"
            anchors {
                top: nameLabel.bottom
                left: parent.left
                right: parent.right
                topMargin: 10
            }
            onClicked: playPauseScanning(ip, scanning)
        }

        Teleop {
            id: teleop
            anchors {
                top: playScanBtn.bottom
                topMargin: 20
                horizontalCenter: parent.horizontalCenter
            }
            onSendTeleop: scanMapListItemFrame.sendTeleop(ip, index)
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
        visible: busy
        radius: 3
        color: Style.lightGreyBackground
        opacity: 0.5
        anchors.fill: parent
    }

    CustomBusyIndicator {
        running: busy
        anchors.fill: parent
    }
}
