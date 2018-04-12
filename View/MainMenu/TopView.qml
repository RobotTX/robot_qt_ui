import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Model/Robot"
import "../Custom"

Frame {
    id: topViewFrame
    property bool hasMap
    property Robots robotModel
    property string langue
    property int mapRotation: Math.round(slider.valueAt(slider.position))
    signal savePosition()
    signal loadPosition()
    padding: 0

    Layout.minimumHeight: Style.menuHeaderHeight
    Layout.maximumHeight: Math.max(Style.menuHeaderHeight, flick.contentItem.childrenRect.height + 15)

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    Flickable {
        id: flick
        ScrollBar.vertical: ScrollBar { }
        contentHeight: contentItem.childrenRect.height
        clip: true
        anchors {
            left: parent.left
            top: parent.top
            right: slider.left
            bottom: parent.bottom
            leftMargin: 10
            topMargin: 5
            bottomMargin: 5
            rightMargin: 5
        }

        MouseArea {
            onWheel: console.log
        }

        Column {
            anchors.left: parent.left
            anchors.right: parent.right
            /*
            Behavior on opacity {
                OpacityAnimator {
                    duration: 250
                }
            }
            */
            CustomLabelWithTimer {
                id: errorLabel
                color: Style.errorColor2
            }
            Label {
                id: warningLabel
                color: Style.warningColor
                visible: text !== ""
                wrapMode: Text.WordWrap
                width: parent.width
            }
            CustomLabelWithTimer {
                id: successLabel
                color: Style.successColor
            }
            CustomLabelWithTimer {
                id: infoLabel
                color: Style.infoColor
            }

            CustomLabelWithTimer {
                id: unmovingLabel
                color: Style.orangeColor
            }

            Repeater {
                model: robotModel
                delegate: Label {
                    color: Style.errorColor2
                    text: "Warning: The robot \"" + name + "\" is running low on battery"
                    visible: battery < 50 * robotModel.batteryWarningThreshold
                    wrapMode: Text.WordWrap
                    width: parent.width
                }
            }
        }
    }

    CustomSlider {
        id: slider

        from: 0
        to: 359
        stepSize: 1

        width: 100

        anchors {
            verticalCenter: saveStateButton.verticalCenter
            right: saveStateButton.left
            rightMargin: 15
        }

        onPositionChanged: console.log("New rotation : " + Math.round(slider.valueAt(slider.position)))
    }

    /// The save state button
    SmallButton {
        id: saveStateButton
        tooltip: langue == "English" ? "保存当前地图位置" : "Save the position of the map"
        imgSrc: "qrc:/icons/saveState"
        anchors {
            top: parent.top
            topMargin: 10
            right: loadStateButton.left
            rightMargin: 14
        }
        enabled: hasMap
        onClicked: topViewFrame.savePosition()
    }

    /// The load state button
    SmallButton {
        id: loadStateButton
        tooltip:  langue == "English" ? "重置当前地图位置" : "Reset the position of the map"
        imgSrc: "qrc:/icons/loadState"
        anchors {
            top: parent.top
            topMargin: 10
            right: parent.right
            rightMargin: 10
        }
        enabled: hasMap
        onClicked: topViewFrame.loadPosition()
    }

    function setMessageTop(label, msg){
        switch(label){
            case 0:
                errorLabel.text = msg;
            break;
            case 1:
                warningLabel.text = msg;
            break;
            case 2:
                successLabel.text = msg;
            break;
            case 3:
                infoLabel.text = msg;
            break;
            case 4:
                unmovingLabel.text = msg;
            break;
            default:
                console.log("Not supposed to be here");
            break;
        }
    }

    function setMapRotation(mapRotation){
        slider.value = mapRotation;
    }
}
