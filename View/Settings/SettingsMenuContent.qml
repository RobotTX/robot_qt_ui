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
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    onVisibleChanged: batterySlider.initializeBatteryThreshold(batteryWarningThreshold)

    Item {

        id: label

        height: 15

        anchors {
            left: parent.left
            top: parent.top
            right: parent.right
        }

        Label {
            id: txt

            anchors {
                left: parent.left
                top: parent.top
            }

            color: "#8F8E94"
            text: qsTr("Laser feedback")
        }

        Button {
            background: Rectangle {
                border.color: Style.lightGreyBorder
                border.width: 1
                radius: 10
            }

            onClicked: console.log("cool")

            anchors.left: txt.right
            anchors.leftMargin: 5
            anchors.top: parent.top

            height: 20
            width: 20
            contentItem: Text {

                text: "?"
                font.pointSize: 12
                font.bold: true
                color: Style.darkSkyBlue

                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
            }
        }

    }

    Flickable {
        id: flick
        height: 80
        ScrollBar.vertical: ScrollBar { }
        contentHeight: contentItem.childrenRect.height
        clip: true
        anchors {
            left: parent.left
            right: parent.right
            top: label.bottom
            topMargin: 10
        }

        Column {
            topPadding: 10
            leftPadding: 50
            spacing: 5
            Repeater {
                id: robots
                model: robotModel
                delegate: CheckDelegate {

                    height: 25

                    id: box
                    anchors.horizontalCenter: parent.horizontalCenter
                    indicator: Image {
                        id: rect
                        anchors.verticalCenter: parent.verticalCenter
                        height: 14
                        width: 14
                        source: box.checked ? "qrc:/icons/valid" : "qrc:/icons/unchecked"
                    }
                    contentItem: Text {
                        id: txt
                        leftPadding: rect.width
                        verticalAlignment: Text.AlignVCenter
                        text: name
                        font.pointSize: 10
                    }



                    checked: true
                }
            }
        }
    }

    Rectangle {
        id: horizontalSeparation
        height: 2
        anchors {
            left: parent.left
            right: parent.right
            top: flick.bottom
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
        anchors.leftMargin: 10

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
        anchors.leftMargin: 12
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

    Button {

        id: cancelButton

        height: 23
        width: 70

        anchors.bottom: parent.bottom
        anchors.left: parent.left

        background: Rectangle {
            radius: 3
            color: cancelButton.pressed ? Style.whiteButtonPressed : "white"
            border.width: 1
            border.color: Style.lightGreyBorder
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

        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom

        onClicked: {
            batteryWarningThreshold = batterySlider.threshold
            console.log("new threshold " + batterySlider.threshold)
        }
    }

    SaveButton {
        id: saveButton
        width: 70
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        onClicked: {
            batteryWarningThreshold = batterySlider.threshold
            console.log("new threshold " + batterySlider.threshold)
            settingsPage.close()
        }
    }
}


