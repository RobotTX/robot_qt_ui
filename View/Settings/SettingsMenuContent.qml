import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Model/Robot"
import "../../View/Custom/"

Frame {

    id: settingsPage
    objectName: "settings"

    signal close()
    signal saveSettingsSignal(int mapChoice, double _batteryThreshold, bool showTutorial)

    property int mapChoice
    property real batteryWarningThreshold: 0.1

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
        // we display at most 3 robots before we start using the scroll bar
        height: Math.min(90, robotModel.count * 30)
        ScrollBar.vertical: ScrollBar { }
        contentHeight: contentItem.childrenRect.height
        clip: true

        anchors {
            left: parent.left
            right: parent.right
            top: label.bottom
            topMargin: 15
        }

        Column {
            topPadding: 10
            leftPadding: 50
            bottomPadding: 10
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
            topMargin: 20
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
        anchors.left: parent.left
        anchors.right: parent.right
    }

    SliderLineMeasurement {
        id: lineMeasurement1
        anchors.top: batterySlider.bottom
        anchors.topMargin: batterySlider.cursor_height-10
        anchors.left: parent.left
        anchors.leftMargin: batterySlider.width/10-batterySlider.cursor_width/2
        txt: "10%"
    }

    SliderLineMeasurement {
        id: lineMeasurement2
        anchors.top: batterySlider.bottom
        anchors.topMargin: batterySlider.cursor_height-10
        anchors.left: lineMeasurement1.right
        anchors.leftMargin: (batterySlider.width/2 - lineMeasurement1.x) / 2 - lineMeasurement1.width - 4
        txt: "20%"
    }

    SliderLineMeasurement {
        id: lineMeasurement3
        anchors.top: batterySlider.bottom
        anchors.topMargin: batterySlider.cursor_height-10
        anchors.left: lineMeasurement2.right
        anchors.horizontalCenter: batterySlider.horizontalCenter
        txt: "30%"
    }

    SliderLineMeasurement {
        id: lineMeasurement4
        anchors.top: batterySlider.bottom
        anchors.topMargin: batterySlider.cursor_height-10
        anchors.left: lineMeasurement3.right
        anchors.leftMargin: 4
        txt: "40%"
    }

    SliderLineMeasurement {
        id: lineMeasurement5
        anchors.top: batterySlider.bottom
        anchors.topMargin: batterySlider.cursor_height-10
        anchors.right: parent.right
        anchors.rightMargin: batterySlider.width/10-batterySlider.cursor_width/2
        txt: "50%"
    }

    Rectangle {
        id: horizontalSeparation3
        height: 2
        anchors {
            top: lineMeasurement1.bottom
            left: parent.left
            right: parent.right
            topMargin: 20
        }
        color: Style.darkGrey2
        opacity: 0.1
    }

    CheckBox {

        id: box2

        property bool show

        anchors.top: horizontalSeparation3.bottom
        anchors.topMargin: 16

        checkable: true
        checked: true

        onClicked: show = !show

        indicator: Image {
            id: rect2
            source: box2.show ? "qrc:/icons/valid" : "qrc:/icons/unchecked"
        }

        contentItem: Text {
            anchors.left: rect2.right
            anchors.leftMargin: 6
            anchors.verticalCenter: rect2.verticalCenter
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
            saveSettingsSignal(mapChoice, batterySlider.threshold, box2.show)
        }
    }

    SaveButton {
        id: saveButton
        width: 70
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        onClicked: {
            batteryWarningThreshold = batterySlider.threshold
            saveSettingsSignal(mapChoice, batterySlider.threshold, box2.show)
            settingsPage.close()
        }
    }

    function setSettings(mapChoice, _thresh, showTutorial){
        settingsPage.batteryWarningThreshold = _thresh;
        box2.show = showTutorial;
        settingsPage.mapChoice = mapChoice;
        switch(mapChoice){
        case 0:
            mapChoice1.checked = true;
            break;
        case 1:
            mapChoice2.checked = true;
            break;
        case 2:
            mapChoice3.checked = true;
            break;
        case 3:
            mapChoice4.checked = true;
            break;
        case 4:
            mapChoice5.checked = true;
            break;
        default:
            break;
        }
    }
}


