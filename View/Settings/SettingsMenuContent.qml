import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Model/Robot"
import "../../View/Custom/"
import "../../Model/Tutorial"

Frame {

    id: settingsPage
    objectName: "settings"

    signal close()
    signal saveSettingsSignal(int mapChoice, double _batteryThreshold)

    property real batteryWarningThreshold
    property int mapChoice

    property real oriBatteryWarningThreshold
    property int oriMapChoice

    property Robots robotModel
    property Tutorial tutorial

    background: Rectangle {
        anchors.fill: parent
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    onVisibleChanged: {
        settingsPage.mapChoice = oriMapChoice;
        settingsPage.batteryWarningThreshold = oriBatteryWarningThreshold;
        batterySlider.initializeBatteryThreshold(batteryWarningThreshold);
    }

    Label {
        id: choiceMapLabel
        anchors {
            left: parent.left
            right: parent.right
            top: parent.top
            topMargin: 5
        }

        color: "#8F8E94"
        text: qsTr("Which map do you want to use ?")
    }

    // the radio buttons to choose which map is used for the robots

    Rectangle {

        id: mapChoices

        height: 130

        anchors {
            top: choiceMapLabel.bottom
            topMargin: 20
            left: parent.left
            leftMargin: 20
        }

        ButtonGroup {
            id: mapChoiceGroup
        }

        RoundCheckBox {
            id: mapChoice1
            ButtonGroup.group: mapChoiceGroup
            checked: mapChoice == 0
            text: qsTr("The robot's map")
            onClicked: mapChoice = 0
        }

        RoundCheckBox {
            id: mapChoice2
            ButtonGroup.group: mapChoiceGroup
            checked: mapChoice == 1

            anchors {
                left: parent.left
                top: mapChoice1.bottom
                topMargin: 12
            }

            text: qsTr("The application's map")
            onClicked: mapChoice = 1
        }

        RoundCheckBox {
            id: mapChoice3
            ButtonGroup.group: mapChoiceGroup
            checked: mapChoice == 2

            anchors {
                left: parent.left
                top: mapChoice2.bottom
                topMargin: 12
            }

            text: qsTr("Always ask me")
            onClicked: mapChoice = 2
        }

        RoundCheckBox {
            id: mapChoice4
            ButtonGroup.group: mapChoiceGroup
            checked: mapChoice == 3
            anchors {
                left: parent.left
                top: mapChoice3.bottom
                topMargin: 12
            }
            text: qsTr("The newest map")
            onClicked: mapChoice = 3
        }

        RoundCheckBox {
            id: mapChoice5
            ButtonGroup.group: mapChoiceGroup
            checked: mapChoice == 4
            anchors {
                left: parent.left
                top: mapChoice4.bottom
                topMargin: 12
            }

            text: qsTr("The oldest map")
            onClicked: mapChoice = 4
        }
    }

    ToolSeparator {
        id: horizontalSeparation2
        orientation: Qt.Horizontal
        anchors {
            top: mapChoices.bottom
            left: parent.left
            right: parent.right
            topMargin: 20
        }
    }

    Item {

        id: batteryLabel

        height: 15

        anchors {
            top: horizontalSeparation2.bottom
            topMargin: 20
        }

        Label {
            id: batteryHelp

            anchors {
                left: parent.left
                top: parent.top
            }

            color: "#8F8E94"
            text: qsTr("Battery level warning trigger")
        }

        Button {

            height: 20
            width: 20

            background: Rectangle {
                border.color: Style.lightGreyBorder
                border.width: 1
                radius: 10
            }

            anchors {
                left: batteryHelp.right
                leftMargin: 5
                top: parent.top
            }

            ToolTip {
                visible: parent.hovered
                text: "Level of battery under which you receive a warning";
                font.pointSize: 10
                x: 26
                y: -4
                background: Rectangle {
                    border.color: Style.darkSkyBlue
                    border.width: 1
                    radius: 8;
                    anchors.fill: parent
                }
            }

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

    BatteryLevelSlider {
        id: batterySlider
        anchors {
            top: batteryLabel.bottom
            topMargin: 16
            left: parent.left
            right: parent.right
        }
    }

    SliderLineMeasurement {
        id: lineMeasurement1
        anchors {
            top: batterySlider.bottom
            topMargin: batterySlider.cursor_height-10
            left: parent.left
            leftMargin: batterySlider.width/10-batterySlider.cursor_width/2
        }
        txt: "10%"
    }

    SliderLineMeasurement {
        id: lineMeasurement2
        anchors {
            top: batterySlider.bottom
            topMargin: batterySlider.cursor_height-10
            left: lineMeasurement1.right
            leftMargin: (batterySlider.width/2 - lineMeasurement1.x) / 2 - lineMeasurement1.width - 4
        }
        txt: "20%"
    }

    SliderLineMeasurement {
        id: lineMeasurement3
        anchors {
            top: batterySlider.bottom
            topMargin: batterySlider.cursor_height-10
            left: lineMeasurement2.right
            horizontalCenter: batterySlider.horizontalCenter
        }
        txt: "30%"
    }

    SliderLineMeasurement {
        id: lineMeasurement4
        anchors {
            top: batterySlider.bottom
            topMargin: batterySlider.cursor_height-10
            left: lineMeasurement3.right
            leftMargin: 4
        }
        txt: "40%"
    }

    SliderLineMeasurement {
        id: lineMeasurement5
        anchors {
            top: batterySlider.bottom
            topMargin: batterySlider.cursor_height-10
            right: parent.right
            rightMargin: batterySlider.width/10-batterySlider.cursor_width/2
        }
        txt: "50%"
    }

    ToolSeparator {
        id: horizontalSeparation3
        orientation: Qt.Horizontal
        anchors {
            top: lineMeasurement1.bottom
            left: parent.left
            right: parent.right
            topMargin: 20
        }
    }

    CancelButton {
        id: cancelButton

        width: 70

        anchors.bottom: parent.bottom
        anchors.left: parent.left

        onClicked: settingsPage.close()
    }

    // apply button to save the changes but keep the window open
    SaveButton {
        id: applyButton

        txt: "Apply"
        width: 70

        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom

        onReleased: {
            batteryWarningThreshold = batterySlider.value;
            saveSettingsSignal(mapChoice, batterySlider.value);
        }
    }

    SaveButton {
        id: saveButton
        width: 70
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        onReleased: {
            batteryWarningThreshold = batterySlider.value;
            saveSettingsSignal(mapChoice, batterySlider.value);
            console.log("save settings signal called " + mapChoice + " " + batterySlider.value);
            settingsPage.close();
        }
    }

    function setSettings(mapChoice){
        console.log("Settings map choice " + mapChoice);
        oriMapChoice = mapChoice;
        settingsPage.mapChoice = mapChoice;
    }
}


