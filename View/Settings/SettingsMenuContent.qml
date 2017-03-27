import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Model/Robot"
import "../../View/Custom/"

Frame {

    id: settingsPage

    signal close()
    signal requestUpdate(real _batteryThreshold)

    property int mapChoice
    property real batteryWarningThreshold

    property Robots robotModel

    background: Rectangle {
        anchors.fill: parent
        color: "#ECECEC"
    }

    onVisibleChanged: batterySlider.initializeBatteryThreshold(batteryWarningThreshold)

    Label {

        id: label

        Text {
            color: "#8F8E94"
            text: qsTr("Laser feedback")
        }

        anchors.top: parent.top
        anchors.left: parent.left

        height: 15

        anchors {
            left: backgroundRectangle.left
            top: backgroundRectangle.top
        }

        Image {
            id: icon
            source: "qrc:/icons/dot"
            fillMode: Image.Pad // to not stretch the image
            anchors{
                verticalCenter: parent.verticalCenter
                left: parent.left
                leftMargin: 120
            }
        }
    }

    Repeater {
        id: robots
        model: robotModel
        delegate: CheckDelegate {
                text: name
                checked: true
        }
    }

    Rectangle {
        id: horizontalSeparation
        height: 2
        anchors {
            top: label.bottom
            left: parent.left
            right: parent.right
            topMargin: 25
        }
        color: Style.darkGrey2
        opacity: 0.1
    }

    Label {
        id: choiceMapLabel
        anchors.top: horizontalSeparation.bottom
        anchors.topMargin: 16
        Text {
            color: "#8F8E94"
            text: qsTr("Which map do you want to use ?")
        }
    }

    Rectangle {

        id: mapChoices
        anchors.top: choiceMapLabel.bottom
        anchors.topMargin: 20
        anchors.left: parent.left

        ButtonGroup {
            id: mapChoiceGroup
        }

        RoundCheckBox {
            id: mapChoice1
            ButtonGroup.group: mapChoiceGroup
            checked: true
            text: qsTr("The robot's map")
            onClicked: mapChoice = 0
        }

        RoundCheckBox {
            id: mapChoice2
            ButtonGroup.group: mapChoiceGroup
            anchors.left: parent.left
            anchors.top: mapChoice1.bottom
            anchors.topMargin: 12
            text: qsTr("The application's map")
            onClicked: mapChoice = 1
        }

        RoundCheckBox {
            id: mapChoice3
            checked: true
            ButtonGroup.group: mapChoiceGroup
            anchors.left: parent.left
            anchors.top: mapChoice2.bottom
            anchors.topMargin: 12
            text: qsTr("Always ask me")
            onClicked: mapChoice = 2
        }

        RoundCheckBox {
            id: mapChoice4
            ButtonGroup.group: mapChoiceGroup
            anchors.left: parent.left
            anchors.top: mapChoice3.bottom
            anchors.topMargin: 12
            text: qsTr("The newest map")
            onClicked: mapChoice = 3
        }

        RoundCheckBox {
            id: mapChoice5
            ButtonGroup.group: mapChoiceGroup
            anchors.left: parent.left
            anchors.top: mapChoice4.bottom
            anchors.topMargin: 12
            text: qsTr("The oldest map")
            onClicked: mapChoice = 4
        }
    }

    Rectangle {
        id: horizontalSeparation2
        height: 2
        anchors {
            top: mapChoices.bottom
            left: parent.left
            right: parent.right
            topMargin: 150
        }
        color: Style.darkGrey2
        opacity: 0.1
    }

    Label {
        id: batteryLabel
        anchors.top: horizontalSeparation2.bottom
        anchors.topMargin: 25
        Text {
            color: "#8F8E94"
            text: qsTr("Battery level warning trigger")
        }
    }

    BatteryLevelSlider {
        id: batterySlider
        anchors.top: batteryLabel.bottom
        anchors.topMargin: 16
    }

    SliderLineMeasurement {
        id: lineMeasurement1
        anchors.top: batterySlider.bottom
        anchors.topMargin: 30
        anchors.left: parent.left
        anchors.leftMargin: 20
        txt: "10%"
    }

    SliderLineMeasurement {
        id: lineMeasurement2
        anchors.top: batterySlider.bottom
        anchors.topMargin: 30
        anchors.left: lineMeasurement1.right
        anchors.leftMargin: 43
        txt: "20%"
    }

    SliderLineMeasurement {
        id: lineMeasurement3
        anchors.top: batterySlider.bottom
        anchors.topMargin: 30
        anchors.left: lineMeasurement2.right
        anchors.leftMargin: 43
        txt: "30%"
    }

    SliderLineMeasurement {
        id: lineMeasurement4
        anchors.top: batterySlider.bottom
        anchors.topMargin: 30
        anchors.left: lineMeasurement3.right
        anchors.leftMargin: 43
        txt: "40%"
    }

    SliderLineMeasurement {
        id: lineMeasurement5
        anchors.top: batterySlider.bottom
        anchors.topMargin: 30
        anchors.left: lineMeasurement4.right
        anchors.leftMargin: 43
        txt: "50%"
    }

    Rectangle {
        id: horizontalSeparation3
        height: 2
        anchors {
            top: lineMeasurement1.bottom
            left: parent.left
            right: parent.right
            topMargin: 45
        }
        color: Style.darkGrey2
        opacity: 0.1
    }

    CheckBox {

        id: box

        anchors.top: horizontalSeparation3.bottom
        anchors.topMargin: 16
        checked: true

        indicator: Image {
            id: rect
            source: box.checked ? "qrc:/icons/valid" : "qrc:/icons/unchecked"
        }

        contentItem: Text {
            anchors.left: rect.right
            anchors.leftMargin: 6
            anchors.verticalCenter: rect.verticalCenter
            anchors.verticalCenterOffset: 1
            text: "Show tutorial"
            font.pointSize: 10
        }
    }

    Rectangle {

        anchors.left: parent.left
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: box.bottom
        anchors.topMargin: 17

        Button {

            id: cancelButton

            height: 23
            width: 70

            background: Rectangle {
                radius: 3
                color: cancelButton.pressed ? Style.whiteButtonPressed : "white"
                border.width: 1
                border.color: "white"
            }

            Label {
                text: qsTr("Cancel")
                color: "Black"
                anchors.verticalCenter: parent.verticalCenter
                anchors.horizontalCenter: parent.horizontalCenter
            }

            onClicked: settingsPage.close()
        }

        Button {

            id: applyButton

            height: 23
            width: 70

            Label {
                text: qsTr("Apply")
                color: "white"
                anchors.verticalCenter: parent.verticalCenter
                anchors.horizontalCenter: parent.horizontalCenter
            }

            background: Rectangle {
                radius: 3
                color: applyButton.pressed ? Style.darkSkyBlueBorder : Style.darkSkyBlue
                border.width: 1
                border.color: Style.darkSkyBlueBorder
            }

            anchors.left:cancelButton.right
            anchors.leftMargin: 10

            onClicked: {
                batteryWarningThreshold = batterySlider.threshold
                console.log("new threshold " + batterySlider.threshold)
            }
        }

        SaveButton {
            width: 70
            anchors.left: applyButton.right
            anchors.leftMargin: 10
            onClicked: {
                batteryWarningThreshold = batterySlider.threshold
                console.log("new threshold " + batterySlider.threshold)
                settingsPage.close()
            }
        }
    }
}


