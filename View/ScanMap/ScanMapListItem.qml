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

    height: 270
    padding: 0
    enabled: !busy

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

        Label {
            id: mapLabel
            text: "Map rotation"
            font.pointSize: 10
            anchors {
                top: playScanBtn.bottom
                left: parent.left
                right: parent.right
                topMargin: 10
            }
            verticalAlignment: Text.AlignVCenter
            height: 21
            width: 80
        }

        TextField {
            id: field

            anchors {
                right: incButton.left
                rightMargin: 8
                verticalCenter: mapLabel.verticalCenter
            }

            background: Rectangle {
                border.color: field.activeFocus ? Style.lightBlue : Style.lightGreyBorder
                border.width: 2
                radius: 2
            }

            height: 21
            width: 40

            padding: 0
            selectByMouse: true
            // range of accepted values : 0 to 359
            validator: IntValidator { bottom: 0; top: 359 }
            inputMethodHints: Qt.ImhDigitsOnly
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignLeft
            placeholderText: "0"
            color: Style.darkSkyBlue
            font.pointSize: 10
            // to update the slider value accordingly
            onAccepted: {
                focus = false;
                slider.value = parseInt(text);
            }
        }

        Button {
            id: incButton

            width: 12
            height: 21

            anchors {
                right: parent.right
                verticalCenter: mapLabel.verticalCenter
            }

            background: Rectangle {
                color: "transparent"
            }

            contentItem: Image {
                anchors.fill: parent
                source: "qrc:/icons/stepper"
                fillMode: Image.PreserveAspectFit

                MouseArea {
                    id: mouseArea
                    anchors.fill: parent

                    // so that you can press the button to increment the value instead of having to click a lot of times
                    Timer {
                        id: timer
                        interval: 50
                        repeat: true
                        onTriggered: {
                            // if we click the lower half of the button we decrement the value of otherwise we increment it
                            if(mouseArea.pressed){
                                if(mouseArea.mouseY > parent.height / 2)
                                    slider.value = slider.value - 1
                                else
                                    slider.value = slider.value + 1
                            }
                        }
                    }

                    onPressed: timer.start()
                    onReleased: timer.stop()
                }
            }
        }

        CustomSlider {
            id: slider

            from: 0
            to: 359
            stepSize: 5

            anchors {
                top: field.bottom
                left: parent.left
                right: parent.right
                topMargin: 10
                leftMargin: 5
                rightMargin: 5
            }

            // to update the text accordingly
            onVisualPositionChanged: field.text = Math.round(valueAt(position))
        }

        Teleop {
            id: teleop
            anchors {
                top: slider.bottom
                topMargin: 20
                horizontalCenter: parent.horizontalCenter
            }
            onSendTeleop: scanMapListItemFrame.sendTeleop(ip, index)
        }

        Rectangle {
            color: Style.lightGreyBorder
            width: parent.width
            height: 2

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
