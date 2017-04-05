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
    signal rotate(int angle, int id)

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
        width: 20
        height: 20

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

        contentItem: Image {
            source: "qrc:/icons/closeBtn"
            anchors.fill: parent
            fillMode: Image.PreserveAspectFit
        }

        onClicked: robotMap.removeMap(ip)
    }

    CustomSlider {
        id: slider

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

        // to update the text accordingly
        onVisualPositionChanged: {
            robotMap.rotate(Math.round(valueAt(position)), index)
            field.text = Math.round(valueAt(position))
        }
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

        TextField {

            id: field

            background: Rectangle {
                border.color: field.activeFocus ? Style.lightBlue : Style.lightGreyBorder
                border.width: 2
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
            anchors {
                left: mapLabel.right
                leftMargin: 4
                top: parent.top
            }
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
                top: parent.top
                left: field.right
                leftMargin: 8
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
