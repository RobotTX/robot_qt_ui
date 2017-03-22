import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQml.Models 2.2
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Path"
import "../Custom"

Frame {
    id: createPathMenuFrame
    objectName: "createPathMenuFrame"
    property string oldName: ""
    property Paths pathModel
    property Paths tmpPathModel

    signal backToMenu()
    signal createPath(string groupName, string name)
    signal createPathPoint(string groupName, string pathName, string name, double x, double y, int waitTime)
    signal useTmpPathModel(bool use)

    Connections {
        target: tmpPathModel
        onValidPositionChanged: enableSave()
    }

    onVisibleChanged: {
        if(!visible){
            if(tmpPathModel && createPathMenuFrame){
                tmpPathModel.clearTmpPath();
                useTmpPathModel(false);
                tmpPathModel.visiblePathChanged();
            }
            oldName = "";
            groupComboBox.currentIndex = 0;
            groupComboBox.displayText = Helper.noGroup;
        } else {
            if(tmpPathModel && createPathMenuFrame){
                tmpPathModel.clearTmpPath();
                useTmpPathModel(true);
                tmpPathModel.visiblePathChanged();
            }
        }
        pathTextField.text = oldName;
    }

    padding: 0

    background: Rectangle {
        z: 2
        anchors.fill: parent
        color: "transparent"
        border.color: Style.lightGreyBorder
        border.width: 1
    }


    Frame {
        id: topFrame
        z: 2
        padding: 0
        height: 230

        anchors {
            left: parent.left
            right: parent.right
            top: parent.top
        }

        background: Rectangle {
            anchors.fill: parent
            color: Style.lightGreyBackground
            border.width: 0
        }

        Label {
            id: pathLabel
            text: qsTr("Path Name")
            color: Style.midGrey2
            anchors {
                left: parent.left
                top: parent.top
                right: parent.right
                leftMargin: 20
                rightMargin: 20
                topMargin: 20
            }
        }

        TextField {
            id: pathTextField
            placeholderText: qsTr("Enter name")
            height: 28
            anchors {
                left: parent.left
                top: pathLabel.bottom
                right: parent.right
                topMargin: 8
                leftMargin: 20
                rightMargin: 20
            }

            background: Rectangle {
                radius: 2
                border.color: pathTextField.activeFocus ? Style.lightBlue : Style.lightGreyBorder
                border.width: pathTextField.activeFocus ? 3 : 1
            }
            onTextChanged: enableSave()
        }

        Label {
            id: groupLabel
            text: qsTr("Choose Group")
            color: Style.midGrey2
            anchors {
                left: parent.left
                top: pathTextField.bottom
                right: parent.right
                topMargin: 20
                leftMargin: 20
                rightMargin: 20
            }
        }

        CustomComboBox {
            id: groupComboBox
            model: pathModel
            displayText: Helper.noGroup //oldName ? oldGroup : Helper.noGroup
            anchors {
                left: parent.left
                top: groupLabel.bottom
                right: parent.right
                topMargin: 8
                leftMargin: 20
                rightMargin: 20
            }
        }

        Label {
            id: pathPointsLabel
            text: qsTr("Path Points")
            color: Style.midGrey2
            anchors {
                left: parent.left
                top: groupComboBox.bottom
                right: parent.right
                topMargin: 20
                leftMargin: 20
                rightMargin: 20
            }
        }

        NormalButton {
            id: addSavedPoint
            txt: "Add Saved Point"
            imgSrc: "qrc:/icons/add"
            anchors {
                left: parent.left
                top: pathPointsLabel.bottom
                right: parent.right
                topMargin: 5
            }
        }

        Rectangle {
            id: space
            color: Style.lightGreyBorder
            width: parent.width
            height: 2
            anchors {
                left: parent.left
                top: addSavedPoint.bottom
                right: parent.right
                topMargin: 5
                leftMargin: 20
                rightMargin: 20
            }
        }
    }

    Frame {
        id: root

        anchors {
            left: parent.left
            top: topFrame.bottom
            right: parent.right
            bottom: bottomFrame.top
            /*topMargin: 5
            bottomMargin: 10
            leftMargin: 20
            rightMargin: 20*/
        }
        padding: 0
        background: Rectangle {
            anchors.fill: parent
            color: Style.lightGreyBackground
            border.width: 0
        }

        Component {
            id: dragDelegate

            MouseArea {
                id: dragArea

                property bool held: false

                anchors {
                    left: parent.left
                    right: parent.right
                    leftMargin: 20
                    rightMargin: 20
                }
                height: content.height + 15

                drag.target: held ? content : undefined
                drag.axis: Drag.YAxis

                onPressAndHold: held = true
                onReleased: held = false

                Rectangle {
                    id: content
                    anchors {
                        horizontalCenter: parent.horizontalCenter
                        verticalCenter: parent.verticalCenter
                    }
                    width: dragArea.width; height: 90 + 4

                    color: dragArea.held ? "lightsteelblue" : "transparent"
                    Behavior on color {
                        ColorAnimation {
                            duration: 100
                        }
                    }

                    radius: 2
                    Drag.active: dragArea.held
                    Drag.source: dragArea
                    Drag.hotSpot.x: width / 2
                    Drag.hotSpot.y: height / 2
                    states: State {
                        when: dragArea.held

                        ParentChange {
                            target: content
                            parent: root
                        }
                        AnchorChanges {
                            target: content
                            anchors {
                                horizontalCenter: undefined
                                verticalCenter: undefined
                            }
                        }
                    }

                    Image {
                        id: validBtn
                        source: validPos ? "qrc:/icons/valid" : "qrc:/icons/notValid"
                        fillMode: Image.Pad
                        anchors {
                            top: parent.top
                            left: parent.left
                            leftMargin: 20
                        }
                    }

                    Label {
                        id: indexId
                        text: index + ":"
                        anchors {
                            left: validBtn.right
                            verticalCenter: validBtn.verticalCenter
                            leftMargin: 4
                        }
                        height: 20
                    }

                    Label {
                        id: nameId
                        text: name
                        anchors {
                            left: indexId.right
                            verticalCenter: validBtn.verticalCenter
                            right: closeBtn.left
                            leftMargin: 6
                        }
                        maximumLineCount: 1
                        elide: Text.ElideRight
                        height: 20
                    }

                    SmallButton {
                        id: closeBtn
                        imgSrc: "qrc:/icons/closeBtn"
                        anchors {
                            verticalCenter: validBtn.verticalCenter
                            right: parent.right
                        }
                    }
                }
                DropArea {
                    anchors {
                        fill: parent
                        margins: 10
                    }

                    onEntered: {
                        tmpPathModel.get(0).paths.get(0).pathPoints.move(drag.source.DelegateModel.itemsIndex,
                                     dragArea.DelegateModel.itemsIndex, 1);
                        tmpPathModel.visiblePathChanged();
                    }
                }

                Rectangle {
                    id: innerSpace
                    color: Style.lightGreyBorder
                    width: parent.width
                    height: 2
                    anchors {
                        left: parent.let
                        right: parent.right
                        top: content.bottom
                        topMargin: 10
                    }
                }
            }
        }

        Repeater {
            model: tmpPathModel
            delegate: Repeater {
                model: paths
                delegate: ListView {
                    id: view
                    ScrollBar.vertical: ScrollBar { }
                    contentHeight: contentItem.childrenRect.height

                    anchors {
                        fill: parent
                        margins: 2
                    }

                    model: pathPoints
                    delegate: dragDelegate

                    spacing: 4
                    cacheBuffer: 50
                }
            }
        }
    }

    Frame {
        id: bottomFrame
        z: 2
        height: 60
        padding: 0

        anchors {
            left: parent.left
            right: parent.right
            bottom: parent.bottom
        }

        background: Rectangle {
            anchors.fill: parent
            color: Style.lightGreyBackground
            border.width: 0
        }

        CancelButton {
            anchors {
                left: parent.left
                right: parent.horizontalCenter
                bottom: parent.bottom
                rightMargin: 5
                leftMargin: 20
                bottomMargin: 20
            }
            onClicked: backToMenu()
        }

        SaveButton {
            id: saveButton
            anchors {
                left: parent.horizontalCenter
                right: parent.right
                bottom: parent.bottom
                leftMargin: 5
                rightMargin: 20
                bottomMargin: 20
            }
            enabled: false
            onClicked: {
                console.log("create " + groupComboBox.displayText + " : " + pathTextField.text);
                createPath(groupComboBox.displayText, pathTextField.text);
                for(var i = 0; i < tmpPathModel.get(0).paths.get(0).pathPoints.count; i++)
                    createPathPoint(groupComboBox.displayText,
                                    pathTextField.text,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).name,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).posX,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).posY,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).waitTime);
                backToMenu();
            }
        }
    }

    function enableSave(){
        var error = false;
        /// TODO need to format the text like in c++

        error = (pathTextField.text === "");

        if(!error)
            for(var i = 0; i < pathModel.get(0).paths.count; i++)
                error = (pathModel.get(0).paths.get(i).pathName === pathTextField.text)

        if(!error)
            error = (tmpPathModel.get(0).paths.get(0).pathPoints.count < 1);

        if(!error)
            for(var j = 0; j < tmpPathModel.get(0).paths.get(0).pathPoints.count; j++)
                if(!error)
                    error = !tmpPathModel.get(0).paths.get(0).pathPoints.get(j).validPos;

        saveButton.enabled = !error;
    }
}
