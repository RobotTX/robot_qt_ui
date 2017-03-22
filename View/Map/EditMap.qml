import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.1
import QtQuick.Layouts 1.3
import QtQml.Models 2.2

import "../../Helper/style.js" as Style
import "../Custom"
import "../../Model/Map/"

Window {

    id: dialog
    width: 1000
    minimumWidth: 800
    height: 700
    minimumHeight: 600

    property string color: "black"
    property string shape: "points"
    property int thickness: 1

    Frame {
        id: rectangleToolbar
        anchors.right: parent.right
        anchors.left: parent.left
        anchors.top: parent.top
        height: undoButton.height
        padding: 0
        z: 2
        background: Rectangle {
            anchors.fill: parent
            color: Style.lightGreyBackground
            border.color: Style.lightGreyBorder
        }

//            border.color: Style.lightGreyBorder

        ToolBar {
            id: toolbar
            anchors.verticalCenter: parent.verticalCenter
            RowLayout {

                ToolButton {
                    id: undoButton
                    Image {
                        source: "qrc:/icons/undo"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                ToolButton {
                    Image {
                        source: "qrc:/icons/redo"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                ToolButton {
/*
                    background: Rectangle {
                        anchors.fill: parent
                        color: parent.pressed ? "red" : Style.lightGreyBackground
                    }
*/
                    Image {
                        source: "qrc:/icons/reset"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                ToolButton {
                    id: selectButton
                    checkable: true
                    checked: true
                    Image {
                        source: "qrc:/icons/hand"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                Rectangle {
                    id: verticalSpaceBar1
                    anchors.left: selectButton.right
                    anchors.leftMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                    color: Style.lightGreyBorder
                    height: selectButton.height - 20
                    width: 2
                }

                ToolButton {
                    id: whiteButton
                    checkable: true

                    Image {
                        source: "qrc:/icons/white"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                    onClicked: {
                        // we implement exclusivity manually as radio buttons are really uggly
                        color = "white"
                        blackButton.checked = false
                        greyButton.checked = false
                    }
                }

                ToolButton {
                    id: greyButton
                    checkable: true
                    Image {
                        source: "qrc:/icons/grey"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                    onClicked: {
                        color = "grey"
                        whiteButton.checked = false
                        blackButton.checked = false
                    }
                }

                ToolButton {
                    id: blackButton
                    checkable: true
                    checked: true
                    Image {
                        source: "qrc:/icons/black"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                    onClicked: {
                        color = "black"
                        greyButton.checked = false
                        whiteButton.checked = false
                    }
                }

                Rectangle {
                    id: verticalSpaceBar2
                    anchors.left: blackButton.right
                    anchors.leftMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                    color: Style.lightGreyBorder
                    height: selectButton.height - 20
                    width: 2
                }

                ToolButton {
                    id: dotButton
                    checkable: true
                    checked: true
                    Image {
                        source: "qrc:/icons/dot"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                    onClicked: {
                        shape = "point"
                        lineButton.checked = false
                        outlineButton.checked = false
                        solidButton.checked = false
                    }
                }

                ToolButton {
                    id: lineButton
                    checkable: true
                    Image {
                        source: "qrc:/icons/line"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                    onClicked: {
                        shape = "line"
                        dotButton.checked = false
                        outlineButton.checked = false
                        solidButton.checked = false
                    }
                }

                ToolButton {
                    id: outlineButton
                    checkable: true
                    Image {
                        source: "qrc:/icons/outline"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                    onClicked: {
                        shape = "outline"
                        lineButton.checked = false
                        dotButton.checked = false
                        solidButton.checked = false
                    }
                }

                ToolButton {
                    id: solidButton
                    checkable: true
                    Image {
                        source: "qrc:/icons/solid"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                    onClicked: {
                        shape = "solid"
                        lineButton.checked = false
                        outlineButton.checked = false
                        dotButton.checked = false
                    }
                }

                Rectangle {
                    id: verticalSpaceBar3
                    anchors.left: solidButton.right
                    anchors.leftMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                    color: Style.lightGreyBorder
                    height: selectButton.height - 20
                    width: 2
                }

                ToolButton {
                    Image {
                        source: "qrc:/icons/decrease"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                ToolButton {
                    Image {
                        source: "qrc:/icons/robot"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                ToolButton {
                    id: increaseButton
                    Image {
                        source: "qrc:/icons/add"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                Rectangle {
                    id: verticalSpaceBar4
                    anchors.left: increaseButton.right
                    anchors.leftMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                    color: Style.lightGreyBorder
                    height: selectButton.height - 20
                    width: 2
                }
            }

            ToolButton {
                id: closeButton
                anchors.right: saveButton.left
                Image {
                    source: "qrc:/icons/closeBtn"
                    anchors.verticalCenter: closeButton.verticalCenter
                    anchors.horizontalCenter: closeButton.horizontalCenter
                }
            }

            ToolButton {
                id: saveButton
                // could not find better than repositionning the button manually (10 is the margin)
                x: dialog.width - 10 - undoButton.width
                Image {
                    source: "qrc:/icons/save"
                    anchors.verticalCenter: saveButton.verticalCenter
                    anchors.horizontalCenter: saveButton.horizontalCenter
                }
            }
        }
    }

    Frame {
        anchors.top: rectangleToolbar.bottom
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        padding: 0
        Image {
            id: img
            smooth: false
            source: "qrc:/maps/map"
            fillMode: Image.PreserveAspectFit
            x: - width / 2 + parent.width / 2
            y: - height / 2 + parent.height / 2

            MouseArea {
                anchors.fill: parent
                onClicked: console.log("clicked the map");
                drag.target: parent
                onWheel: {
                    var newScale = img.scale + img.scale * wheel.angleDelta.y / 120 / 10;
                    if(newScale > Style.minZoom && newScale < Style.maxZoom)
                        img.scale = newScale;
                }
            }

            Canvas {
                smooth: false
                id: canvas
                anchors.top: img.top
                anchors.right: parent.right
                anchors.left: parent.left
                anchors.bottom: parent.bottom
                contextType: "2d"

                renderStrategy: Canvas.Threaded

                property var points: []

                property int startLineX
                property int startLineY

                EditMapModel {
                    id: items
                }

                onPaint: {
                    var ctx = getContext('2d')
                    for(var i = 0; i < items.count; i++){
                        //console.log("number items " + items.count)
                        ctx.fillStyle = "red";
                        if(items.get(i).shape === "points"){
                            //console.log("Number of points in this item " + items.get(i).points.count);
                            for(var j = 0; j < items.get(i).points.count; j += 2){
                                //console.log("point : " + items.get(i).points.get(j).x + " " + items.get(i).points.get(j+1).y);
                                ctx.fillRect(items.get(i).points.get(j).x-3, items.get(i).points.get(j).y-3, 3, 3);
                            }
                        }
                        if(items.get(i).shape === "line"){
                            //console.log("Number of points in this item line " + items.get(i).points.count)
                            //console.log("About to draw a line from " + items.get(i).points.get(0).x + " " + items.get(i).points.get(0).y + " to " +
                            //            items.get(i).points.get(1).x + " " + items.get(i).points.get(1).y);
                            ctx.lineTo(items.get(i).points.get(0).x, items.get(i).points.get(0).y);
                            ctx.stroke();
                        }
                    }
                }



                MouseArea {

                    anchors.fill: parent
                    propagateComposedEvents: true

                    onPressed: {
                        if(selectButton.checked)
                            mouse.accepted = false;
                        else {
                            mouse.accepted = true;
                            if(shape === "points"){
                                console.log("Gonna draw a bunch of points")
                                canvas.points.push(Math.round(mouseX))
                                canvas.points.push(Math.round(mouseY))
                            }
                            if(shape === "line"){
                                console.log("Gonna draw a line")
                                canvas.startLineX = Math.round(mouseX)
                                canvas.startLineY = Math.round(mouseY)
                            }
                            items.addItem(shape, thickness, color, canvas.points, true)
                            canvas.requestPaint()
                        }
                    }

                    onPositionChanged: {
                        if(!selectButton.checked){
                            canvas.points.push(Math.round(mouseX))
                            canvas.points.push(Math.round(mouseY))
                            items.addItem(shape, thickness, color, canvas.points, true)
                            canvas.requestPaint()
                        }
                    }

                    onReleased: {
                        if(shape === "line")
                            canvas.points = [canvas.startLineX, canvas.startLineY, Math.round(mouseX), Math.round(mouseY)];
                        items.addItem(shape, thickness, color, canvas.points, false)
                        canvas.requestPaint()
                        canvas.points = []
                    }
                }
            }
        }
    }
}
