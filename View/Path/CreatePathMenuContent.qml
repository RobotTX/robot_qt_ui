import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQml.Models 2.2
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Path"
import "../../Model/Point"
import "../../Model/Speech"
import "../Point"
import "../Speech"
import "../Custom"

Frame {
    id: createPathMenuFrame
    objectName: "createPathMenuFrame"
    property string oldName // when editing
    property string oldGroup // when editing
    property string errorMsg
    property string robotPathName // when creating a new path from robot
    property ListModel robotPathPoints // when creating a new path from robot
    property bool nameError: true
    property Paths pathModel
    property Paths tmpPathModel
    property Points pointModel
    property Speechs speechModel
    property string langue
    property int menuIndex: 0

    signal backToMenu()
    signal createPath(string groupName, string name)
    signal createPathPoint(string groupName, string pathName, string name, double x, double y, int waitTime, int orientation, string speechName, string speechContent, int speechTime)
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
            robotPathName = "";
            var displayTextChange = "";
            if (langue == "English") {
                displayTextChange = Helper.noGroupChinese;
            } else {
                displayTextChange = Helper.noGroup;
            }

//            if (robotPathPoints)
//                robotPathPoints.clear();
            groupComboBox.currentIndex = 0;
            groupComboBox.displayText = displayTextChange;
        } else {
            if(tmpPathModel && createPathMenuFrame){
                tmpPathModel.clearTmpPath();
                useTmpPathModel(true);
                tmpPathModel.visiblePathChanged();

                // if editing
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
                                            "orientation": pathModel.get(i).paths.get(j).pathPoints.get(k).orientation,
                                            "validPos": true,
                                            "speechName": pathModel.get(i).paths.get(j).pathPoints.get(k).speechName,
                                            "speechContent": pathModel.get(i).paths.get(j).pathPoints.get(k).speechContent,
                                            "speechTime": pathModel.get(i).paths.get(j).pathPoints.get(k).speechTime
                                       });
                                    }
                                }
                            }
                        }
                    } // if creating path from robot
                } else if (robotPathName !== "") {
                    for(var l = 0; l < robotPathPoints.count; l++){
                        /// speechName will always be empty when user would save a path
                        tmpPathModel.get(0).paths.get(0).pathPoints.append({
                            "name": robotPathPoints.get(l).pathPointName,
                            "posX": robotPathPoints.get(l).pathPointPosX,
                            "posY": robotPathPoints.get(l).pathPointPosY,
                            "waitTime": robotPathPoints.get(l).waitTime,
                            "orientation": robotPathPoints.get(l).orientation,
                            "validPos": true,
                            "speechName": ""
                       });
                    }
                }
            }
        }

        pathTextField.text = (oldName !== "" ? oldName : robotPathName);

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
        height: 250

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
            text: langue == "English" ? "路径名称" : qsTr("Path Name")
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
            placeholderText: langue == "English" ? "输入路径名称" : qsTr("Enter name")
            verticalAlignment: TextInput.AlignVCenter
            anchors {
                left: parent.left
                top: pathLabel.bottom
                right: parent.right
                topMargin: 8
                leftMargin: 20
                rightMargin: 20
            }
/*
            onEditingFinished: {
                if(saveButton.canSave)
                    saveButton.released()
            }
*/
            wrapMode: Text.WordWrap
            background: Rectangle {
                radius: 2
                border.color: nameError ? Style.errorColor : pathTextField.activeFocus ? Style.lightBlue : Style.lightGreyBorder
                border.width: pathTextField.activeFocus || nameError ? 3 : 1
            }
            onTextChanged: enableSave()
        }

        Label {
            id: groupLabel
            text: langue == "English" ? "选择分组" : qsTr("Choose Group")
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
            langue: createPathMenuFrame.langue
            displayText: {
                if (oldname) {
                    oldGroup
                } else {
                    langue == "English" ? Helper.noGroupChinese : Helper.noGroup
                }
            }
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
            text: langue == "English" ? qsTr("路径目标点") : qsTr("Path Points")
            font.pointSize: 12
            color: Style.midGrey2
            anchors {
                left: parent.left
                top: groupComboBox.bottom
                right: parent.right
                topMargin: langue == "English" ? 13 : 20
                leftMargin: 20
                rightMargin: 20
            }
        }

        NormalButton {
            id: addSavedPoint
            txt: langue == "English" ? "加入已有目标点" : "Add Saved Point"
            sizeTxt: "11"
            imgSrc: "qrc:/icons/point_checked_2525"
            anchors {
                left: parent.left
                top: pathPointsLabel.bottom
                right: parent.right
                topMargin: 0
            }
            onClicked: {
                pointList.open()
            }
        }

        PointListInPopup {
            id: pointList
            pointModel: createPathMenuFrame.pointModel
            x: addSavedPoint.width
            y: addSavedPoint.y
            menuIndex: createPathMenuFrame.menuIndex
            onPointSelected: {
                tmpPathModel.addPathPoint(name,  "tmpPath", "tmpGroup", posX, posY, 0, orientation, "", "", 0);
                tmpPathModel.checkTmpPosition(tmpPathModel.get(0).paths.get(0).pathPoints.count - 1, posX, posY);
                tmpPathModel.visiblePathChanged();
            }
        }

        ToolSeparator {
            id: space
            orientation: Qt.Horizontal
            anchors {
                left: parent.left
                top: addSavedPoint.bottom
                right: parent.right
                topMargin: langue === "English" ? -10 : 5
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
//            topMargin: langue === "English" ? 40 : 0
//            topMargin: 0
            right: parent.right
            bottom: bottomFrame.top
        }
        padding: 0
        background: Rectangle {
            anchors.fill: parent
            color: Style.lightGreyBackground
//            color: "transparent"
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
                    width: dragArea.width
                    height: 230

                    color: dragArea.held ? Style.lightBlue : "transparent"

                    radius: 2
                    /*Drag.active: dragArea.held
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
                    }*/

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
                        property string nameCoordinates: Math.round(posX) + " " + Math.round(posY)
                        id: nameId
                        text: name === nameCoordinates ? "P"+(index+1) : name

                        selectByMouse: true
                        color: "#262626"
                        padding: 2
                        anchors {
                            left: indexId.right
                            verticalCenter: validBtn.verticalCenter
                            right: closeBtn.left
                            leftMargin: 6
                        }
//                        height: langue === "English" ? 28 : 20
                        verticalAlignment: TextInput.AlignVCenter
                        background: Rectangle {
                            radius: 2
                            border.color: nameId.activeFocus ? Style.lightBlue : Style.lightGreyBorder
                            border.width: nameId.activeFocus ? 3 : 1
                        }
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
                        text: langue == "English" ? qsTr("人为干预") : qsTr("Human Action")
                        checked: waitTime < 0 ? true : false
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
                        text: langue == "English" ? qsTr("等待") : qsTr("Wait for")
                        checked: waitTime >= 0 ? true : false
                        anchors {
                            left: indexId.left
                            top: humanAction.bottom
                            topMargin: 11
                        }
                        onCheckedChanged: {
                            humanAction.checked = !waitFor.checked
                            if(waitFor.checked && waitTime < 0)
                               waitTextField.text = "0"
                        }
                    }



                    TextField {
                        id: waitTextField
                        selectByMouse: true
                        text: waitTime
                        height: 20
                        width: 40
                        padding: 2
                        horizontalAlignment: TextInput.AlignRight

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
                        text: langue == "English" ? "秒" : "sec(s)"
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

                    Label {
                        id: oriLabel
                        text: langue == "English" ? qsTr("方向") : qsTr("Orientation")
                        font.pointSize: 10
                        color: Style.greyText
                        anchors {
                            left: indexId.left
                            top: minText.bottom
                            topMargin: 15
                        }
                        width: 80
                    }

                    TextField {
                        id: field
                        background: Rectangle {
                            border.color: field.activeFocus ? Style.lightBlue : Style.lightGreyBorder
                            border.width: field.activeFocus ? 3 : 1
                        }

                        height: 20
                        width: 40
                        padding: 2

                        selectByMouse: true
                        // range of accepted values : 0 to 359
                        validator: IntValidator { bottom: 0; top: 359 }
                        inputMethodHints: Qt.ImhDigitsOnly
                        verticalAlignment: Text.AlignVCenter
                        horizontalAlignment: TextInput.AlignRight
                        placeholderText: "0"

                        text: orientation
                        Component.onCompleted: slider.value = orientation

                        anchors {
                            left: waitTextField.left
                            verticalCenter: oriLabel.verticalCenter

                        }

                        // to update the slider value accordingly
                        onAccepted: {
                            focus = false;
                            slider.value = parseInt(text);
                        }
                    }

                    Button {
                        id: incButtonBis

                        width: 12
                        height: 21

                        anchors {
                            left: field.right
                            leftMargin: 4
                            verticalCenter: oriLabel.verticalCenter
                        }

                        background: Rectangle {
                            color: "transparent"
                        }

                        contentItem: Image {
                            anchors.fill: parent
                            source: "qrc:/icons/stepper"
                            fillMode: Image.PreserveAspectFit

                            MouseArea {
                                id: mouseAreaBis
                                anchors.fill: parent

                                // so that you can press the button to increment the value instead of having to click a lot of times
                                Timer {
                                    id: timerBis
                                    interval: 50
                                    repeat: true
                                    onTriggered: {
                                        // if we click the lower half of the button we decrement the value of otherwise we increment it
                                        if(mouseAreaBis.pressed){
                                            if(mouseAreaBis.mouseY > parent.height / 2)
                                                slider.value = slider.value - 1
                                            else
                                                slider.value = slider.value + 1
                                        }
                                    }
                                }

                                onPressed: timerBis.start()

                                onReleased: timerBis.stop()
                            }
                        }
                    }

                    CustomSlider {
                        id: slider
                //        visible: homeCheckBox.checked

                        from: 0
                        to: 359
                        stepSize: 1

                        anchors {
                            left: oriLabel.left
                            top: oriLabel.bottom
                            right: parent.right
                            rightMargin: 25
                            topMargin: 10
                        }
                        onPositionChanged: {
                            tmpPathModel.setOrientation("tmpGroup", "tmpPath", index, Math.round(slider.valueAt(slider.position)));
//                            console.log("index : " + index);
                            field.text = Math.round(slider.valueAt(slider.position));
                        }
                    }

                    NormalButton {
                        id: addSpeech
                        txt: langue == "English" ? "加入已有语音" : "Add Speech"
                        imgSrc: "qrc:/icons/add_speech"
                        anchors {
                            left: parent.left
                            top: slider.bottom
                            right: parent.right
                            topMargin: 8
                        }
                        font.pointSize: 11
                        onClicked: speechList.open()
                    }

                    SpeechListInPopup {
                        id: speechList
                        speechModel: createPathMenuFrame.speechModel
                        x: addSpeech.width
                        y: addSpeech.y
                        menuIndex: createPathMenuFrame.menuIndex
                        onSpeechSelected: {
                            tmpPathModel.setSpeechInfos("tmpGroup", "tmpPath", index, nameSpeech, tts);
                        }
                    }

                    Label {
                        id: speechLabel
                        visible: speechName !== ""
                        text: langue == "English" ? "名称 : " : "Name : "
                        font.pointSize: 10
                        color: Style.greyText
                        anchors {
                            left: waitFor.left
                            top: addSpeech.bottom
                            topMargin: 8
                        }
                    }

                    Label {
                        id: speechNameLabel
                        visible: speechName !== ""
                        text: speechName
                        font.pointSize: 10
                        color: Style.greyText
                        anchors {
                            left: speechLabel.right
                            bottom: speechLabel.bottom
                            leftMargin: 5
                            verticalCenter: speechLabel.verticalCenter
                        }
                    }

                    SmallButton {
                        id: closeBtnSpeech
                        visible: speechName !== ""
                        imgSrc: "qrc:/icons/closeBtn"
                        anchors {
                            left: speechNameLabel.right
                            leftMargin: 4
                            verticalCenter: speechNameLabel.verticalCenter
                            right: parent.right
                        }
                        onClicked: {
                            tmpPathModel.setSpeechInfos("tmpGroup", "tmpPath", index, "", "");
                        }
                    }

                    Label {
                        id: speechTimeLabel
                        visible: speechName !== ""
                        text: langue == "English" ? "等待" : "Wait for"
                        anchors {
                            top: speechLabel.bottom
                            left: speechLabel.left
                            topMargin: 10
                        }
                        font.pointSize: 10
                    }

                    TextField {
                        id: speechTimeTextField
                        visible: speechName !== ""
                        selectByMouse: true
                        text: speechTime
                        height: 20
                        width: 40
                        padding: 2
                        horizontalAlignment: TextInput.AlignRight

                        validator: IntValidator{bottom: 0; top: 999;}
                        anchors {
                            left: speechTimeLabel.right
                            verticalCenter: speechTimeLabel.verticalCenter
                            leftMargin: 4
                        }
                        enabled: speechLabel

                        background: Rectangle {
                            radius: 2
                            border.color: speechTimeTextField.activeFocus ? Style.lightBlue : Style.lightGreyBorder
                            border.width: speechTimeTextField.activeFocus ? 3 : 1
                        }
                        onFocusChanged: !focus && text === "" ? text = "0" : undefined
                        onTextChanged: text === "" ? speechTime = 0 : speechTime = parseInt(text)
                    }

                    Button {
                        id: incButton2
                        visible: speechName !== ""
                        width: 12
                        height: 21

                        anchors {
                            left: speechTimeTextField.right
                            verticalCenter: speechTimeTextField.verticalCenter
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
                                id: mouseArea2
                                anchors.fill: parent

                                // so that you can press the button to increment the value instead of having to click a lot of times
                                Timer {
                                    id: timer2
                                    interval: 50
                                    repeat: true
                                    onTriggered: {
                                        // if we click the lower half of the button we decrement the value of otherwise we increment it
                                        if(mouseArea2.pressed){
                                            if(mouseArea2.mouseY > parent.height / 2)
                                                speechTimeTextField.text = Math.max(0 , (parseInt(speechTimeTextField.text) - 1)).toString()
                                            else
                                                speechTimeTextField.text = Math.min((parseInt(speechTimeTextField.text) + 1)).toString()
                                        }
                                    }
                                }

                                onPressed: timer2.start()

                                onReleased: timer2.stop()
                            }
                        }
                    }

                    Label {
                        id: secText
                        visible: speechName !== ""
                        text: langue == "English" ? "秒" : "sec(s)"
                        font.pointSize: 10
                        color: Style.greyText
                        anchors {
                            left: incButton2.right
                            verticalCenter: incButton2.verticalCenter
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

                ToolSeparator {
                    id: innerSpace
                    orientation: Qt.Horizontal
                    width: parent.width
                    anchors {
                        left: parent.let
                        right: parent.right
                        top: content.bottom
                        topMargin: langue === "English" ? 15 : 10
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
                    ScrollBar.vertical: ScrollBar {
                        policy: ScrollBar.AlwaysOn
                    }
                    contentHeight: contentItem.childrenRect.height

                    anchors {
                        fill: parent
                        margins: 2
                    }

                    model: pathPoints
                    delegate: dragDelegate

                    spacing: 10
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
            langue: createPathMenuFrame.langue
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
            langue: createPathMenuFrame.langue
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
                var groupName = groupComboBox.displayText
                if (groupComboBox.displayText === Helper.noGroupChinese) {
                    groupComboBox.displayText = Helper.noGroup;
                }

                if(oldName !== "") {
                    pathModel.deletePath(oldGroup, oldName);
//                    console.log("oldname != '' ", oldName)
                }
                createPath(groupComboBox.displayText, newName);

                var mess1 = ''
                var mess2 = ''
                if (langue == "English") {
                    mess1 = "已创建路径 \"" + newName + "\" 在 \"" + groupName + "\""
//                    mess2 = "编辑路径 \"" + oldName + "\" 从 \"" + oldGroup + "\" 到 \"" + newName + "\" 到 \"" + groupName + "\""
                } else {
                    mess1 = "Created the path \"" + newName + "\" in \"" + groupName + "\""
//                    mess2 = "Edited a path from \"" + oldName + "\" in \"" + oldGroup + "\" to \"" + newName + "\" in \"" + groupName + "\""
                }

                for(var i = 0; i < tmpPathModel.get(0).paths.get(0).pathPoints.count; i++) {
                    createPathPoint(groupComboBox.displayText,
                                    newName,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).name,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).posX,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).posY,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).waitTime,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).orientation,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).speechName,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).speechContent,
                                    tmpPathModel.get(0).paths.get(0).pathPoints.get(i).speechTime
                                    );
                }

//                setMessageTop(3, oldName === "" ? mess1 : mess2)
                setMessageTop(3, mess1)
                backToMenu();
            }
        }
    }

    function enableSave(){
        var newName = Helper.formatName(pathTextField.text);

        errorMsg = "";

        var error = (newName === "");
        var mess1 = ''
        if (langue == "English") {
            mess1 = "路径名不能为空"
        } else {
            mess1 = "The path name cannot be empty"
        }

        if(error)
            errorMsg = mess1

        if(newName !== "" && newName !== oldName) {
            var mess2 = ''
            if (langue == "English") {
                mess2 = "路径名" + newName + "已经被使用"
            } else {
                mess2  ="The path name \"" + newName + "\" is already taken"
            }

        // This is available in all editors.
            for(var i = 0; i < pathModel.count; i++)
                for(var j = 0; j < pathModel.get(i).paths.count; j++){
                    if(pathModel.get(i).paths.get(j).pathName === newName){
                        errorMsg += (errorMsg !== "" ? "\n" : "") + mess2;
                        error = true;
                    }
                }
        }
        nameError = error;

        var mess3 = ''
        if (langue == "English") {
            mess3 = "至少需要一个目标点来创建路径"
        } else {
            mess3 = "You need at least 1 point to create a path"
        }

        if(tmpPathModel.get(0).paths.get(0).pathPoints.count < 1){
            errorMsg += (errorMsg !== "" ? "\n" : "") + mess3;
            error = true;
        }

        var mess4 = ''
        for(var k = 0; k < tmpPathModel.get(0).paths.get(0).pathPoints.count; k++){
            if(!tmpPathModel.get(0).paths.get(0).pathPoints.get(k).validPos){
                if (langue == "English") {
                    mess3 =  "目标点 " + (k+1) + " 在错误的位置，机器人无法去到那里"
                } else {
                    mess3 =  "The point " + (k+1) + " is at a wrong position, your robot(s) would not be able to go there"
                }
                errorMsg += (errorMsg !== "" ? "\n" : "") + mess3;
                error = true;
            }
        }

        setMessageTop(1, errorMsg);
        saveButton.canSave = !error;
    }
}
