import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Window 2.2
import QtQml.Models 2.2

import "../../Helper/style.js" as Style
import "../Custom"
import "../../Model/Map/"
import "../../Model/Tutorial"
import "../Tutorial"

Window {

    id: dialog
    title: langue == "English" ? "编辑地图" : "Edit Map"
    objectName: "editMapWindow"

    width: 1000
    minimumWidth: 800
    height: 700
    minimumHeight: 600

    property Tutorial tutorial
    property string langue
    property string imgSource

    property var imagesArray: ["qrc:/icons/line1", "qrc:/icons/line2", "qrc:/icons/line3", "qrc:/icons/line4",
                       "qrc:/icons/line5", "qrc:/icons/line6", "qrc:/icons/line7", "qrc:/icons/line8"]

    // properties of the items drawn on the map
    property color color: "#000000"
    property int shape: 0
    property int thickness: 1

    property int mapRotation: Math.round(slider.valueAt(slider.position))

    signal clicked(int shape, color color, int thickness, int x, int y, bool update)
    signal resetMap()
    signal undo()
    signal redo()
    signal saveImage(string location, int mapRot)
    signal savePosition(double posX, double posY, double zoom, int mapRotation, string mapSrc)
    signal orientationMap(int orientationMap)
    // to clear the map of its items everytime we show the window
    onVisibleChanged: {
        // to reset the map on c++ side
        if(visible)
            dialog.resetMap();
        // to reset the toolbar
        shape = -1;
        color = "#000000";
        thickness = 1;
        if(!visible)
            tutorialD.close();
        else
//            if(tutorial.isDisplayed("edit_map"))
//                tutorialD.open();
            langue == "English" ? tutorialDChinese.open() : tutorialD.open()
    }

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

            CustomToolTip { text: langue == "English" ? "撤销" : "Undo" }

            Shortcut {
                sequence: StandardKey.Undo
                onActivated: dialog.undo()
            }

            src: "qrc:/icons/undo"

            anchors {
                verticalCenter: parent.verticalCenter
                left: parent.left
                leftMargin: 7
            }

            onClicked: dialog.undo()
        }

        EditMapToolButton {

            id: redo

            CustomToolTip { text: langue == "English" ? "恢复" :  "Redo" }

            Shortcut {
                sequence: "Ctrl+Y"
                onActivated: dialog.redo()
            }

            src: "qrc:/icons/redo"

            anchors {
                verticalCenter: parent.verticalCenter
                left: undo.right
                leftMargin: 7
            }

            onClicked: dialog.redo()
        }

        EditMapToolButton {

            id: reset

            CustomToolTip { text: langue == "English" ? "撤销所有修改" : "Cancel all modifications" }

            src: "qrc:/icons/reset"

            anchors {
                verticalCenter: parent.verticalCenter
                left: redo.right
                leftMargin: 7
            }

            onClicked: dialog.resetMap()
        }

        EditMapToolButton {

            id: selectButton

            CustomToolTip { text: langue == "English" ? "拖动地图" : "Drag the map" }

            //checkable: true
            checked: shape == -1
            src: "qrc:/icons/hand"

            anchors {
                verticalCenter: parent.verticalCenter
                left: reset.right
                leftMargin: 7
            }
            onClicked: shape = -1
        }

        ToolSeparator {
            id: verticalSpaceBar1
            anchors {
                verticalCenter: parent.verticalCenter
                left: selectButton.right
                leftMargin: 7
            }
        }


        EditMapToolButton {

            id: whiteButton

            //checkable: true
            checked: color == "#ffffff" // white

            CustomToolTip { text: langue == "English" ? "加入自由区域到地图" : "Add a known area to the map" }

            src: "qrc:/icons/white"

            anchors {
                verticalCenter: parent.verticalCenter
                left: verticalSpaceBar1.right
                leftMargin: 7
            }

            onClicked: color = "#ffffff" // white
        }

        EditMapToolButton {

            id: greyButton

            //checkable: true
            checked: color == Style.mapGrey

            CustomToolTip { text: langue == "English" ? "加入未知区域到地图" : "Add an unknown area to the map" }

            src: "qrc:/icons/grey"

            anchors {
                verticalCenter: parent.verticalCenter
                left: whiteButton.right
                leftMargin: 7
            }

            onClicked: color = Style.mapGrey
        }

        EditMapToolButton {

            id: blackButton

            //checkable: true
            checked: color == "#000000" // black

            CustomToolTip { text: langue == "English" ? "加入障碍物区域到地图" : "Add an obstacle to the map" }

            src: "qrc:/icons/black"

            anchors {
                verticalCenter: parent.verticalCenter
                left: greyButton.right
                leftMargin: 7
            }

            onClicked: color = "#000000" // black
        }

        ToolSeparator {
            id: verticalSpaceBar2
            anchors {
                verticalCenter: parent.verticalCenter
                left: blackButton.right
                leftMargin: 7
            }
        }


        EditMapToolButton {

            id: dotButton

            CustomToolTip { text: langue == "English" ? "画点到地图" : "Draw a point on the map" }

            //checkable: true
            checked: shape == 0

            src: "qrc:/icons/dot"

            anchors {
                verticalCenter: parent.verticalCenter
                left: verticalSpaceBar2.right
                leftMargin: 7
            }

            onClicked: shape = 0
        }

        EditMapToolButton {

            id: lineButton

            CustomToolTip { text: langue == "English" ? "画线到地图" : "Draw a line on the map" }

            //checkable: true
            checked: shape == 1

            src: "qrc:/icons/line"

            anchors {
                verticalCenter: parent.verticalCenter
                left: dotButton.right
                leftMargin: 7
            }

            onClicked: shape = 1
        }

        EditMapToolButton {

            id: outlineButton

            CustomToolTip { text: langue == "English" ? "画空心矩形到地图" : "Draw an empty rectangle on the map" }

            //checkable: true
            checked: shape == 2

            src: "qrc:/icons/outline"

            anchors {
                verticalCenter: parent.verticalCenter
                left: lineButton.right
                leftMargin: 7
            }

            onClicked: shape = 2
        }

        EditMapToolButton {

            id: solidButton

            CustomToolTip { text: langue == "English" ? "画实心矩形到地图" : "Draw a filled rectangle on the map" }

            //checkable: true
            checked: shape == 3

            src: "qrc:/icons/solid"

            anchors {
                verticalCenter: parent.verticalCenter
                left: outlineButton.right
                leftMargin: 7
            }

            onClicked: shape = 3
        }

        ToolSeparator {
            id: verticalSpaceBar3
            anchors {
                verticalCenter: parent.verticalCenter
                left: solidButton.right
                leftMargin: 7
            }
        }

        EditMapToolButton {

            id: decrease

            CustomToolTip { text: langue == "English" ? "减小画笔尺寸" : "Decrease the size of your brush" }

            src: "qrc:/icons/decrease"

            anchors {
                verticalCenter: parent.verticalCenter
                left: verticalSpaceBar3.right
                leftMargin: 7
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
                leftMargin: 7
            }
        }

        EditMapToolButton {

            id: increaseButton

            CustomToolTip { text: langue == "English" ? "增大画笔尺寸" : "Increase the size of your brush" }

            src: "qrc:/icons/increase_brush"

            anchors {
                verticalCenter: parent.verticalCenter
                left: thicknessImage.right
                leftMargin: 7
            }

            onClicked: {
                if(thickness < 8)
                    thickness++;
            }
        }

        ToolSeparator {
            id: verticalSpaceBar4
            anchors {
                verticalCenter: parent.verticalCenter
                left: increaseButton.right
                leftMargin: 7
            }
        }

        CustomSlider {
            id: slider

            from: 0
            to: 359
            stepSize: 1

            width: 100

            anchors {
                left: verticalSpaceBar4.right
                leftMargin: 7
                verticalCenter: parent.verticalCenter
            }

            onPositionChanged: {
                orientationMap(Math.round(slider.valueAt(slider.position)));
                console.log("orientation = " + Math.round(slider.valueAt(slider.position)));
            }
        }

        ToolSeparator {
            id: verticalSpaceBar5
            anchors {
                verticalCenter: parent.verticalCenter
                left: slider.right
                leftMargin: 7
            }
        }

        Button {
            id: helpButton

            height: 24
            width: 24

            background: Rectangle {
                border.color: Style.lightGreyBorder
                border.width: 1
                radius: 12
                color: "white"
            }

            anchors {
                left: verticalSpaceBar5.right
                leftMargin: 7
                verticalCenter: parent.verticalCenter
            }

            contentItem: Label {
                text: "?"
                font.pointSize: 12
                font.bold: true
                color: Style.darkSkyBlue
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
            }

            CustomToolTip { text: langue == "English" ? "提示" : "Help" }

            onClicked: langue == "English" ? tutorialDChinese.open() : tutorialD.open()
        }

        EditMapToolButton {

            id: cancelButton

            CustomToolTip { text: langue == "English" ? "取消修改" : "Cancel modifications" }

            src: "qrc:/icons/closeBtn"

            anchors {
                verticalCenter: parent.verticalCenter
                right: saveButton.left
            }

            onClicked: {
                dialog.hide();
            }
        }

        EditMapToolButton {

            id: saveButton

            CustomToolTip { text: langue == "English" ? "保存修改到当前地图" : "Save your modifications to the current map and notify your robots" }

            src: "qrc:/icons/save"

            anchors {
                verticalCenter: parent.verticalCenter
                right: parent.right
                rightMargin: 7
            }

            onClicked: {
                dialog.hide();
                var toto = "";
                var sub0 = imgSource.substring(0);
                var i = 6;
                console.log("imgSource.substring(0) = " + imgSource.substring(0));
                console.log("imgSource.substring(1) = " + imgSource.substring(1));
                console.log("imgSource.substring(2) = " + imgSource.substring(2));
                console.log("imgSource.substring(3) = " + imgSource.substring(3));
                console.log("imgSource.substring(5) = " + imgSource.substring(5));
                console.log("imgSource.substring("+ i +") = " + imgSource.substring(i) + "<--- the one using right now");
//                var toto = imgSource.substring(0);
//                if (imgSource.substring(0).indexOf("file:/")) {
//                    toto = toto.split("file:").join(":");
//                    console.log("toto = " + toto);
//                } else if (imgSource.substring(0).indexOf("file:C:")) {
//                    toto = toto.split("file:C:").join(":");
//                    console.log("toto = " + toto);
//                }
                if (Qt.platform.os === "windows") {
                    console.log("windows platform");
                    if (sub0.indexOf("file:/") !== -1) {
                        toto = imgSource.substring(4);
                        console.log("+++++ we are in first case, substring 5");
                    } else if (sub0.indexOf("file:C:") !== -1) {
                        toto = imgSource.substring(6);
                        console.log("+++++ we are in second case, susbtring 6");
                    }
                } else {
                    console.log("other platforms");
                    toto = imgSource.substring(6);
                }

                console.log("toto = " + toto);
//                toto = imgSource.substring(i);
//                dialog.saveImage(imgSource.substring(4), mapRotation);
                dialog.saveImage(toto, mapRotation);
//                console.log("mapRotation = " + mapRotation);
//                mapRotation = 0;
//                console.log("mapRotation = " + mapRotation);
            }
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
//            onSourceChanged: console.log("edit map source changed " + image.source)
            fillMode: Image.PreserveAspectFit

            x: - width / 2 + parent.width / 2
            y: - height / 2 + parent.height / 2

            rotation: dialog.mapRotation

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
                }

                // when we drag we don't want to create a new item, we want to add more points to the last item (so that undo and redo functions erase or repaint the whole acceptedButtons
                // group of points together
                onPositionChanged: {
                    if(!selectButton.checked)
                        dialog.clicked(shape, color, thickness, mouseX, mouseY, true)
                }

                drag.target: {

                    selectButton.checked ? parent: undefined
                }

                onWheel: {
                    var newScale = image.scale + image.scale * wheel.angleDelta.y / 120 / 10;
                    if(newScale > Style.minZoom && newScale < Style.maxZoom)
                        image.scale = newScale;
                }
            }              
        }
    }

    TutorialDialog {
        id: tutorialD
        height: 500
        x: dialog.width / 2 - width / 2
        y: dialog.height / 2 - height / 2
        feature: "EDIT MAP"
        tutorial: dialog.tutorial
        langue: dialog.langue
        Component.onCompleted: tutoMessage = tutorial.getMessage("EDIT MAP")
    }

    TutorialDialog {
        id: tutorialDChinese
        height: 500
        x: dialog.width / 2 - width / 2
        y: dialog.height / 2 - height / 2
        feature: "编辑地图"
        tutorial: dialog.tutorial
        langue: dialog.langue
        Component.onCompleted: tutoMessage = tutorial.getMessage("编辑地图")
    }

}
