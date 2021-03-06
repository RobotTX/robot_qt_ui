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
    property string mapSrc
    property int mapRotation: Math.round(slider.valueAt(slider.position))
    signal savePosition()
    signal zoomInMap()
    signal zoomOutMap()
    signal centerMapTopView()
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
            onWheel: {}
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
            verticalCenter: zoomInButton.verticalCenter
            right: zoomInButton.left
            rightMargin: 15
        }

        onPositionChanged: console.log("New rotation : " + Math.round(slider.valueAt(slider.position)))
//        onPositionChanged: {}
    }

    /// Zoom in button
    SmallButton {
        id: zoomInButton
        tooltip: langue == "English" ? "Zoom in" : "地图放大"
        imgSrc: "qrc:/icons/zoomIn"
        anchors {
            top: parent.top
            topMargin: 7
            right: zoomOutButton.left
            rightMargin: 14
        }
        enabled: hasMap
        onClicked: topViewFrame.zoomInMap()
    }

    /// Zoom out button
    SmallButton {
        id: zoomOutButton
        tooltip: langue == "English" ? "Zoom out" : "地图缩小"
        imgSrc: "qrc:/icons/zoomOut"
        anchors {
            top: parent.top
            topMargin: 7
            right: centerMapButton.left
            rightMargin: 14
        }
        enabled: hasMap
        onClicked: topViewFrame.zoomOutMap()
    }

    /// Center map button
    SmallButton {
        id: centerMapButton
        tooltip:  langue == "English" ? "Map Center Position" : "地图中心位置"
        imgSrc: "qrc:/icons/centerMap"
        anchors {
            top: parent.top
            topMargin: 7
            right: parent.right
            rightMargin: 10
        }

        onClicked: topViewFrame.centerMapTopView()
    }


    property string mapFileName: mapSrc.toString()

    CustomLabel {
        id: mapName
        text: {
            var indexLastSlash = "";
            if (mapFileName.indexOf("/") !== -1) {
                    indexLastSlash = mapFileName.lastIndexOf("/");
              } else {
                  indexLastSlash = mapFileName.lastIndexOf("\\");
              }
            qsTr(mapFileName.substring(indexLastSlash + 1));
        }
        anchors {
            top: slider.bottom
            right: parent.right
            topMargin: 4
            rightMargin: 10
        }
        font.pointSize: Style.ubuntuSubTextSize
        color: Style.midGrey
        font.italic: true

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
