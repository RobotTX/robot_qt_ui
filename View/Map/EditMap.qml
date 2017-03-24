import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Window 2.2
import QtQml.Models 2.2

import "../../Helper/style.js" as Style
import "../Custom"
import "../../Model/Map/"

Window {

    id: dialog
    objectName: "editMapWindow"

    width: 1000
    minimumWidth: 800
    height: 700
    minimumHeight: 600

    property string imgSource

    property var imagesArray: ["qrc:/icons/add", "qrc:/icons/grey", "qrc:/icons/black", "qrc:/icons/undo",
                       "qrc:/icons/white", "qrc:/icons/reset", "qrc:/icons/redo", "qrc:/icons/hand"]

    // properties of the items drawn on the map
    property color color: "black"
    property int shape: 0
    property int thickness: 1

    signal clicked(int shape, color color, int thickness, int x, int y, bool update)
    signal resetMap()
    signal undo()
    signal redo()
    signal saveImage(string location)

    // to clear the map of its items everytime we show the window
    onVisibleChanged: dialog.resetMap()

    Frame {

        id: toolbar

        background: Rectangle {
            anchors.fill: parent
            color: Style.lightGreyBackground
        }

        // to place the toolbar on top of the map
        z: 2

        height: 40
        padding: 0

        anchors {
            left: parent.left
            top: parent.top
            right: parent.right
        }

        EditMapToolButton {

            id: undo

            EditMapToolTip { text: "Undo" }

            Shortcut {
                sequence: StandardKey.Undo
                onActivated: dialog.undo()
            }

            src: "qrc:/icons/undo"

            anchors {
                verticalCenter: parent.verticalCenter
                left: parent.left
                leftMargin: 10
            }

            onClicked: dialog.undo()
        }

        EditMapToolButton {

            id: redo

            EditMapToolTip { text: "Redo" }

            Shortcut {
                sequence: "Ctrl+Y"
                onActivated: dialog.redo()
            }

            src: "qrc:/icons/redo"

            anchors {
                verticalCenter: parent.verticalCenter
                left: undo.right
                leftMargin: 10
            }

            onClicked: dialog.redo()
        }

        EditMapToolButton {

            id: reset

            EditMapToolTip { text: "Cancel all modifications" }

            src: "qrc:/icons/reset"

            anchors {
                verticalCenter: parent.verticalCenter
                left: redo.right
                leftMargin: 10
            }

            onClicked: dialog.resetMap()
        }

        EditMapToolButton {

            id: selectButton

            EditMapToolTip { text: "Drag the map" }

            checkable: true

            src: "qrc:/icons/hand"

            anchors {
                verticalCenter: parent.verticalCenter
                left: reset.right
                leftMargin: 10
            }
        }

        Rectangle {

            id: verticalSpaceBar1

            color: Style.lightGreyBorder
            width: 2
            height: undo.height - 20

            anchors {
                verticalCenter: parent.verticalCenter
                left: selectButton.right
                leftMargin: 10
            }
        }

        // to create a group of mutually exclusive buttons for all the different colors
        ButtonGroup { id: colorGroup }

        EditMapToolButton {

            id: whiteButton

            ButtonGroup.group: colorGroup

            checkable: true
            checked: false

            EditMapToolTip { text: "Add a known area to the map" }

            src: "qrc:/icons/white"

            anchors {
                verticalCenter: parent.verticalCenter
                left: verticalSpaceBar1.right
                leftMargin: 10
            }

            onClicked: color = "white"
        }

        EditMapToolButton {

            id: greyButton

            checkable: true
            checked: false

            ButtonGroup.group: colorGroup

            EditMapToolTip { text: "Add an unknown area to the map" }

            src: "qrc:/icons/grey"

            anchors {
                verticalCenter: parent.verticalCenter
                left: whiteButton.right
                leftMargin: 10
            }

            onClicked: {
                color = Style.mapGrey
            }
        }

        EditMapToolButton {

            id: blackButton

            checkable: true
            checked: true

            ButtonGroup.group: colorGroup

            EditMapToolTip { text: "Add an obstacle to the map" }

            src: "qrc:/icons/black"

            anchors {
                verticalCenter: parent.verticalCenter
                left: greyButton.right
                leftMargin: 10
            }

            onClicked: color = "black"
        }

        Rectangle {

            id: verticalSpaceBar2

            color: Style.lightGreyBorder
            width: 2
            height: undo.height - 20

            anchors {
                verticalCenter: parent.verticalCenter
                left: blackButton.right
                leftMargin: 10
            }
        }

        // to create a group of mutually exclusive buttons for all the different shapes
        ButtonGroup { id: shapeGroup }

        EditMapToolButton {

            id: dotButton

            EditMapToolTip { text: "Draw a point on the map" }

            checkable: true
            checked: true

            src: "qrc:/icons/dot"

            anchors {
                verticalCenter: parent.verticalCenter
                left: verticalSpaceBar2.right
                leftMargin: 10
            }

            ButtonGroup.group: shapeGroup

            onClicked: shape = 0
        }

        EditMapToolButton {

            id: lineButton

            EditMapToolTip { text: "Draw a line on the map" }

            checkable: true

            src: "qrc:/icons/line"

            anchors {
                verticalCenter: parent.verticalCenter
                left: dotButton.right
                leftMargin: 10
            }

            ButtonGroup.group: shapeGroup

            onClicked: shape = 1
        }

        EditMapToolButton {

            id: outlineButton

            EditMapToolTip { text: "Draw an empty rectangle on the map" }

            ButtonGroup.group: shapeGroup

            checkable: true

            src: "qrc:/icons/outline"

            anchors {
                verticalCenter: parent.verticalCenter
                left: lineButton.right
                leftMargin: 10
            }

            onClicked: shape = 2
        }

        EditMapToolButton {

            id: solidButton

            EditMapToolTip { text: "Draw an filled rectangle on the map" }

            checkable: true

            src: "qrc:/icons/solid"

            ButtonGroup.group: shapeGroup

            anchors {
                verticalCenter: parent.verticalCenter
                left: outlineButton.right
                leftMargin: 10
            }

            onClicked: shape = 3
        }

        Rectangle {

            id: verticalSpaceBar3

            color: Style.lightGreyBorder
            width: 2
            height: solidButton.height - 20

            anchors {
                verticalCenter: parent.verticalCenter
                left: solidButton.right
                leftMargin: 10
            }
        }

        EditMapToolButton {

            id: decrease

            EditMapToolTip { text: "Decrease the size of your brush" }

            src: "qrc:/icons/decrease"

            anchors {
                verticalCenter: parent.verticalCenter
                left: verticalSpaceBar3.right
                leftMargin: 10
            }

            onClicked: {
                if(thickness > 1)
                    thickness--;
            }
        }

        Image {

            id: thicknessImage

            source: imagesArray.length < thickness ? "qrc:/icons/robot" : imagesArray[thickness-1]

            anchors {
                verticalCenter: parent.verticalCenter
                left: decrease.right
                leftMargin: 10
            }
        }

        EditMapToolButton {

            id: increaseButton

            EditMapToolTip { text: "Increase the size of your brush" }

            src: "qrc:/icons/add"

            anchors {
                verticalCenter: parent.verticalCenter
                left: thicknessImage.right
                leftMargin: 10
            }

            onClicked: {
                if(thickness < 8)
                    thickness++;
            }
        }

        Rectangle {

            id: verticalSpaceBar4

            color: Style.lightGreyBorder
            width: 2
            height: increaseButton.height - 20

            anchors {
                verticalCenter: parent.verticalCenter
                left: increaseButton.right
                leftMargin: 10
            }
        }

        EditMapToolButton {

            id: closeButton

            EditMapToolTip { text: "Close the current window: all modifications will be lost" }

            src: "qrc:/icons/closeBtn"

            anchors {
                verticalCenter: parent.verticalCenter
                right: saveButton.left
                rightMargin: 10
            }

            onClicked: dialog.hide();
        }

        EditMapToolButton {

            id: saveButton

            EditMapToolTip { text: "Save your modifications to the current map and notify your robots" }

            src: "qrc:/icons/save"

            anchors {
                verticalCenter: parent.verticalCenter
                right: parent.right
                rightMargin: 10
            }

            onClicked: dialog.saveImage(imgSource.substring(6));
        }
    }

    Frame {

        anchors.top: toolbar.bottom
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        padding: 0

        Image {

            id: image

            smooth: false
            source: imgSource
            fillMode: Image.PreserveAspectFit

            x: - width / 2 + parent.width / 2
            y: - height / 2 + parent.height / 2

            MouseArea {

                objectName: "editMapImage"

                anchors.fill: parent

                onPressed: {
                    if(!selectButton.checked)
                        dialog.clicked(shape, color, thickness, mouseX, mouseY, false);
                }

                onClicked: {
                    if(!selectButton.checked)
                        dialog.clicked(shape, color, thickness, mouseX, mouseY, true);
                    console.log("clicked the map" + mouseX + " " + mouseY);
                }

                // when we drag we don't want to create a new item, we want to add more points to the last item (so that undo and redo functions erase or repaint the whole acceptedButtons
                // group of points together
                onPositionChanged: {
                    if(!selectButton.checked)
                        dialog.clicked(shape, color, thickness, mouseX, mouseY, true)
                }

                drag.target: selectButton.checked ? parent: undefined // gros menteur

                onWheel: {
                    var newScale = image.scale + image.scale * wheel.angleDelta.y / 120 / 10;
                    if(newScale > Style.minZoom && newScale < Style.maxZoom)
                        image.scale = newScale;
                }
            }              
        }
    }
}
