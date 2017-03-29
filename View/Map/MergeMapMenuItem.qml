import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import QtQml 2.2

import "../../Model/Robot"
import "../Custom"
import "../../Helper/style.js" as Style

Rectangle {

    id: robotMap

    anchors {
        left: parent.left
        right: parent.right
    }

    Layout.preferredHeight: 95
    Layout.preferredWidth: parent.width
    color: "transparent"

    signal removeMap(string ip)

    Label {
        id: label
        font.pointSize: 11
        text: qsTr(name)
        verticalAlignment: Text.AlignVCenter
        anchors {
            left: parent.left
            leftMargin: 0
            top: parent.top
            topMargin: 13
        }
        width: 60
        maximumLineCount: 1
        elide: Text.ElideRight
    }

    Button {
        background: Rectangle {
            anchors.fill: parent
            color: "transparent"
        }

        anchors {
            top: parent.top
            topMargin: 13
            right: parent.right
            rightMargin: 15
        }

        width: 20
        height: 20
        contentItem: Image {
            source: "qrc:/icons/closeBtn"
            anchors.fill: parent
            fillMode: Image.PreserveAspectFit
        }
        onClicked: robotMap.removeMap(ip)
    }

    Slider {
        id: slider
        padding: 0
        from: 0
        to: 359
        stepSize: 5
        anchors {
            top: label.bottom
            topMargin: 20
            left: parent.left
            leftMargin: 10
            right: parent.right
            rightMargin: 10
        }
        height: 14
        handle: Rectangle {
            id: handle
            anchors {
                top: parent.top
            }
            radius: 10
            height: 14
            width: 14
            color: Style.darkSkyBlue
            x: slider.visualPosition * slider.availableWidth - width / 2
            y: slider.availableHeight / 2 - height / 2
        }
        background: Rectangle {
            id: background
            x: slider.leftPadding
            y: slider.availableHeight / 2 - height / 2
            height: 2
            width: slider.availableWidth
            color: "#bdbebf"
            radius: 2

            Rectangle {
                width: handle.x
                height: parent.height
                color: Style.darkSkyBlue
                radius: 2
            }
        }
        onVisualPositionChanged: field.text = Math.round(valueAt(position))
    }

    Rectangle {
        anchors {
            top: slider.bottom
            bottom: parent.bottom
            topMargin: 17
            left: parent.left
            right: parent.right

        }
        color: "transparent"

        Label {
            id: mapLabel
            text: "Map rotation"
            font.pointSize: 10
            anchors {
                left: parent.left
                top: parent.top
            }
            verticalAlignment: Text.AlignVCenter
            height: 21
            width: 80
        }

        Rectangle {
            id: editField
            border.color: "white"
            border.width: 1
            height: 21
            width: 40
            anchors {
                left: mapLabel.right
                leftMargin: 4
                top: parent.top

            }

            TextField {
                id: field
                padding: 0
                selectByMouse: true
                validator: IntValidator { bottom: 0; top: 359 }
                inputMethodHints: Qt.ImhDigitsOnly
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignLeft
                placeholderText: "0"

                color: Style.darkSkyBlue
                font.pointSize: 10
                anchors {
                    fill: parent
                }
                onAccepted: {
                    focus = false;
                    slider.value = parseInt(text);
                }
            }
        }

        Button {
            id: incButton

            background: Rectangle {
                anchors.fill: parent

                MouseArea {
                    id: mouseArea
                    anchors.fill: parent
                    Timer {
                        id: timer
                        interval: 50
                        repeat: true
                        onTriggered: {
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

            anchors {
                top: parent.top
                left: editField.right
                leftMargin: 8
            }
            width: 12
            height: 21
            contentItem: Image {
                anchors.fill: parent
                source: "qrc:/icons/stepper"
                fillMode: Image.PreserveAspectFit
            }
        }

        Rectangle {
            anchors {
                top: mapLabel.bottom
                topMargin: 17
                left: parent.left
                leftMargin: 10
                right: parent.right
                rightMargin: 10
            }
            color: Style.lightGreyBorder
            height: 2
        }
    }
}
