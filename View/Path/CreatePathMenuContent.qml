import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQml.Models 2.2
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Path"
import "../../Model/Point"
import "../Point"
import "../Custom"

Frame {
    id: createPathMenuFrame
    objectName: "createPathMenuFrame"
    property string oldName
    property string oldGroup
    property string errorMsg
    property bool nameError: true
    property Paths pathModel
    property Paths tmpPathModel
    property Points pointModel

    signal backToMenu()
    signal createPath(string groupName, string name)
    signal createPathPoint(string groupName, string pathName, string name, double x, double y, int waitTime)
    signal useTmpPathModel(bool use)
    signal setMessageTop(int status, string msg)

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
            oldGroup = "";
            groupComboBox.currentIndex = 0;
            groupComboBox.displayText = Helper.noGroup;
        } else {
            if(tmpPathModel && createPathMenuFrame){
                tmpPathModel.clearTmpPath();
                useTmpPathModel(true);
                tmpPathModel.visiblePathChanged();

                if(oldName !== ""){
                    for(var i = 0; i < pathModel.count; i++){
                        if(pathModel.get(i).groupName === oldGroup){
                            groupComboBox.currentIndex = i;
                            groupComboBox.displayText = oldGroup;
                            for(var j = 0; j < pathModel.get(i).paths.count; j++){
                                if(pathModel.get(i).paths.get(j).pathName === oldName){
                                    for(var k = 0; k < pathModel.get(i).paths.get(j).pathPoints.count; k++){
                                        tmpPathModel.get(0).paths.get(0).pathPoints.append({
                                            "name": pathModel.get(i).paths.get(j).pathPoints.get(k).name,
                                            "posX": pathModel.get(i).paths.get(j).pathPoints.get(k).posX,
                                            "posY": pathModel.get(i).paths.get(j).pathPoints.get(k).posY,
                                            "waitTime": pathModel.get(i).paths.get(j).pathPoints.get(k).waitTime,
                                            "validPos": true
                                       });
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        pathTextField.text = oldName;
        errorMsg = "";
        setMessageTop(1, errorMsg);
    }

    padding: 0

    background: Rectangle {
        z: 2
        anchors.fill: parent
        color: "transparent"
        border.color: Style.lightGreyBorder
        border.width: 1
    }


    // The top frame with the path name, the group and the button to add a path point from a saved point
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
            selectByMouse: true
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

            onEditingFinished: {
                if(saveButton.canSave)
                    saveButton.clicked()
            }

            background: Rectangle {
                radius: 2
                border.color: nameError ? Style.errorColor : pathTextField.activeFocus ? Style.lightBlue : Style.lightGreyBorder
                border.width: pathTextField.activeFocus || nameError ? 3 : 1
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
            displayText: oldName ? oldGroup : Helper.noGroup
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
            onClicked: pointList.open()
        }

        PointListInPopup {
            id: pointList
            pointModel: createPathMenuFrame.pointModel
            x: addSavedPoint.width
            y: addSavedPoint.y
            onPointSelected: {
                console.log(name + " " + posX + " " + posY)
                tmpPathModel.addPathPoint(name,  "tmpPath", "tmpGroup", posX, posY, 0);
                tmpPathModel.checkTmpPosition(tmpPathModel.get(0).paths.get(0).pathPoints.count - 1, posX, posY);
                tmpPathModel.visiblePathChanged();
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

    /// The middle frame with the list of path points
    Frame {
        id: pathPointList

        anchors {
            left: parent.left
            top: topFrame.bottom
            right: parent.right
            bottom: bottomFrame.top
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
                hoverEnabled: true

                anchors {
                    left: parent.left
                    right: parent.right
                    leftMargin: 20
                    rightMargin: 20
                }
                height: content.height + 15

                drag.target: held ? content : undefined
                drag.axis: Drag.YAxis

                onPressed: held = true
                onReleased: held = false

                Rectangle {
                    id: content
                    anchors {
                        horizontalCenter: parent.horizontalCenter
                        verticalCenter: parent.verticalCenter
                    }
                    width: dragArea.width; height: 90 + 4

                    color: dragArea.held ? Style.lightBlue : "transparent"

                    radius: 2
                    Drag.active: dragArea.held
                    Drag.source: dragArea
                    Drag.hotSpot.x: width / 2
                    Drag.hotSpot.y: height / 2
                    states: State {
                        when: dragArea.held

                        ParentChange {
                            target: content
                            parent: pathPointList
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
                            topMargin: 5
                        }
                    }

                    Label {
                        id: indexId
                        text: index + 1 + "."
                        color: "#262626"
                        anchors {
                            left: validBtn.right
                            verticalCenter: validBtn.verticalCenter
                            leftMargin: 4
                        }
                        height: 20
                    }

                    TextField {
                        id: nameId
                        text: name
                        selectByMouse: true
                        color: "#262626"
                        padding: 2
                        anchors {
                            left: indexId.right
                            verticalCenter: validBtn.verticalCenter
                            right: closeBtn.left
                            leftMargin: 6
                        }
                        height: 20
                        background: Rectangle {
                            radius: 2
                            border.color: nameId.activeFocus ? Style.lightBlue : Style.lightGreyBorder
                            border.width: nameId.activeFocus ? 3 : 1
                        }
                        onFocusChanged: !focus && text === "" ? name = Math.round(posX) + ' ' + Math.round(posY) : undefined
                        onTextChanged: name = text
                    }

                    SmallButton {
                        id: closeBtn
                        imgSrc: "qrc:/icons/closeBtn"
                        anchors {
                            verticalCenter: validBtn.verticalCenter
                            right: parent.right
                        }
                        onClicked: {
                            tmpPathModel.get(0).paths.get(0).pathPoints.remove(index);
                            tmpPathModel.visiblePathChanged();
                            enableSave();
                        }
                    }

                    RoundCheckBox {
                        id: humanAction
                        text: qsTr("Human Action")
                        checked: false
                        anchors {
                            left: indexId.left
                            top: indexId.bottom
                            topMargin: 10
                        }
                        onCheckedChanged: {
                            waitTime = humanAction.checked ? -1 : parseInt(waitTextField.text);
                            waitFor.checked = !humanAction.checked;
                        }
                    }

                    RoundCheckBox {
                        id: waitFor
                        text: qsTr("Wait for")
                        checked: true
                        anchors {
                            left: indexId.left
                            top: humanAction.bottom
                            topMargin: 11
                        }
                        onCheckedChanged: {
                            humanAction.checked = !waitFor.checked
                        }
                    }

                    TextField {
                        id: waitTextField
                        selectByMouse: true
                        text: "0"
                        height: 20
                        width: 30
                        padding: 2
                        validator: IntValidator{bottom: 0; top: 999;}
                        anchors {
                            left: waitFor.right
                            verticalCenter: waitFor.verticalCenter
                        }
                        enabled: waitFor.checked

                        background: Rectangle {
                            radius: 2
                            border.color: waitTextField.activeFocus ? Style.lightBlue : Style.lightGreyBorder
                            border.width: waitTextField.activeFocus ? 3 : 1
                        }
                        onFocusChanged: !focus && text === "" ? text = "0" : undefined
                        onTextChanged: text === "" ? waitTime = 0 : waitTime = parseInt(text)
                    }

                    Button {
                        id: incButton

                        width: 12
                        height: 21

                        anchors {
                            left: waitTextField.right
                            verticalCenter: waitTextField.verticalCenter
                            leftMargin: 4
                        }

                        background: Rectangle {
                            color: "transparent"
                        }

                        contentItem: Image {
                            anchors.fill: parent
                            source: "qrc:/icons/stepper"
                            fillMode: Image.PreserveAspectFit

                            MouseArea {
                                id: mouseArea
                                anchors.fill: parent

                                // so that you can press the button to increment the value instead of having to click a lot of times
                                Timer {
                                    id: timer
                                    interval: 50
                                    repeat: true
                                    onTriggered: {
                                        // if we click the lower half of the button we decrement the value of otherwise we increment it
                                        if(mouseArea.pressed){
                                            if(mouseArea.mouseY > parent.height / 2)
                                                waitTextField.text = Math.max(0 , (parseInt(waitTextField.text) - 1)).toString()
                                            else
                                                waitTextField.text = Math.min((parseInt(waitTextField.text) + 1)).toString()
                                        }
                                    }
                                }

                                onPressed: timer.start()

                                onReleased: timer.stop()
                            }
                        }
                    }

                    Label {
                        id: minText
                        text: "Mins"
                        font.pointSize: 10
                        color: Style.greyText
                        anchors {
                            left: incButton.right
                            verticalCenter: waitFor.verticalCenter
                            leftMargin: 4
                        }
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
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
                }
            }
        }
    }

    /// The bottom frame with the buttons to save or cancel
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
            tooltip: errorMsg
            canSave: false
            onReleased: if(saveButton.canSave) {
                var newName = Helper.formatName(pathTextField.text);
                console.log("create " + groupComboBox.displayText + " : " + newName);
                if(oldName !== "")
                    pathModel.deletePath(oldGroup, oldName);
                createPath(groupComboBox.displayText, newName);
                for(var i = 0; i < tmpPathModel.get(0).paths.get(0).pathPoints.count; i++)
                    createPathPoint(groupComboBox.displayText,
                                    newName,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).name,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).posX,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).posY,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).waitTime);
                setMessageTop(2, oldName === "" ? "Created the path \"" + newName + "\" in \"" + groupComboBox.displayText + "\"" :
                                                "Edited a path from \"" + oldName + "\" in \"" + oldGroup + "\" to \"" + newName + "\" in \"" + groupComboBox.displayText + "\"")
                backToMenu();
            }
        }
    }

    function enableSave(){
        var newName = Helper.formatName(pathTextField.text);

        errorMsg = "";

        var error = (newName === "");
        if(error)
            errorMsg = "The path name can not be empty";

        if(newName !== "" && !newName !== oldName)
            for(var i = 0; i < pathModel.count; i++)
                for(var j = 0; j < pathModel.get(i).paths.count; j++){
                    if(pathModel.get(i).paths.get(j).pathName === newName){
                        errorMsg += (errorMsg !== "" ? "\n" : "") + "The path name \"" + newName + "\" is already taken";
                        error = true;
                    }
                }

        nameError = error;

        if(tmpPathModel.get(0).paths.get(0).pathPoints.count < 1){
            errorMsg += (errorMsg !== "" ? "\n" : "") + "You need at least 1 point to create a path";
            error = true;
        }

        for(var k = 0; k < tmpPathModel.get(0).paths.get(0).pathPoints.count; k++){
            if(!tmpPathModel.get(0).paths.get(0).pathPoints.get(k).validPos){
                errorMsg += (errorMsg !== "" ? "\n" : "") + "The point " + (k+1) + " is at a wrong position, your robot(s) would not be able to go there";
                error = true;
            }
        }

        setMessageTop(1, errorMsg);
        saveButton.canSave = !error;
    }
}
