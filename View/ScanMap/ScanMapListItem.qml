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
            enabled: connected
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
            enabled: connected
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
        id: connectedRect
        visible: !connected
        radius: 3
        color: Style.lightGreyBackground
        opacity: 0.5
        anchors.fill: parent
        anchors.topMargin: 35
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
