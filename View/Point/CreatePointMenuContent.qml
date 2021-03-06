import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Controls.Styles 1.4
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../../Model/Robot"
import "../../Model/Path"
import "../Custom"
import "../Robot"

Frame {
    id: createPointMenuFrame
    objectName: "createPointMenuFrame"
    property Points pointModel
    property PointView tmpPointView
    property Paths pathModel
    property string langue
    property string oldName: ""
    property string oldGroup
    property bool nameError: true
    property bool wasDisplayed: false
    property string errorMsg
    property string homeName // when creating a new charging station from robot
    property string homeX // when creating a new charging station from robot
    property string homeY // when creating a new charging station from robot
    property string homeOri // when creating a new charging station from robot
    property Robots robotModel

    signal backToMenu()
    signal createPoint(string name, string groupName, double x, double y, string oldName, string oldGroup, bool displayed, bool home, int orientation)
    signal checkPoint(string name, string oldName, double x, double y)
    signal setMessageTop(int status, string msg)

    Connections {
        target: tmpPointView
        onTmpPointViewPosChanged: {
            checkPoint(Helper.formatName(pointTextField.text), oldName,
                       tmpPointView.x,
                       tmpPointView.y)
        }
    }

    onVisibleChanged: {

        homeCheckBox.checked = false;
        slider.value = 0;

        if(tmpPointView && createPointMenuFrame){
            tmpPointView._isVisible = visible;
            tmpPointView.x =0;
            tmpPointView.y = 0;
        }

        if(!visible){
            /// When you finish or cancel an edition, we show the point you were editing
            if(oldName !== ""){
                for(var i = 0; i < pointModel.count; i++)
                    if(pointModel.get(i).groupName === oldGroup)
                        for(var j = 0; j < pointModel.get(i).points.count; j++)
                            if(pointModel.get(i).points.get(j).name === oldName)
                                pointModel.get(i).points.setProperty(j, "isVisible", true);
            }
            oldName = "";
            oldGroup = "";

            var displayTextChange = "";
            if (langue == "English") {
                displayTextChange = Helper.noGroup;
            } else {
                displayTextChange = Helper.noGroupChinese;
            }
            groupComboBox.currentIndex = 0;
            groupComboBox.displayText = displayTextChange;
        } else {
            // if editing
            if(oldName !== ""){
                for(var i = 0; i < pointModel.count; i++)
                    if(pointModel.get(i).groupName === oldGroup)
                        for(var j = 0; j < pointModel.get(i).points.count; j++)
                            if(pointModel.get(i).points.get(j).name === oldName){
                                wasDisplayed = pointModel.get(i).points.get(j).isVisible;
                                pointModel.get(i).points.setProperty(j, "isVisible", false);
                                homeCheckBox.checked = pointModel.get(i).points.get(j).home;
                                slider.value = pointModel.get(i).points.get(j).orientation;

                                tmpPointView.setType(pointModel.get(i).points.get(j).home ? Helper.PointViewType.HOME_TEMP : Helper.PointViewType.TEMP);
                                tmpPointView.x = pointModel.get(i).points.get(j).posX;
                                tmpPointView.y = pointModel.get(i).points.get(j).posY;
                                tmpPointView.setOrientation(pointModel.get(i).points.get(j).orientation);
                                groupComboBox.currentIndex = i;
                                groupComboBox.displayText = oldGroup;
                            }
            // if creating point from robot (home)
            } else if (homeName !== "") {
                homeCheckBox.checked = true;
                slider.value = homeOri;
                tmpPointView.x = homeX;
                tmpPointView.y = homeY;
                tmpPointView.setOrientation(homeOri);
            }
        }
        pointTextField.text = oldName;
        errorMsg = "";
        setMessageTop(1, errorMsg);
    }

    padding: 20

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    Label {
        id: pointLabel
        text: langue == "English" ? qsTr("Point Name") : qsTr("目标点名称")
        font.pointSize: Style.ubuntuSubHeadingSize
        color: Style.midGrey2
        anchors {
            left: parent.left
            top: parent.top
        }
    }

    TextField {
        id: pointTextField
        selectByMouse: true
        placeholderText: langue == "English" ? qsTr("Enter name") : qsTr("输入目标点名称")
        font.pointSize: Style.ubuntuSubHeadingSize
        text: oldName
        verticalAlignment: TextInput.AlignVCenter
        anchors {
            left: parent.left
            top: pointLabel.bottom
            right: parent.right
            topMargin: 8
        }
        onVisibleChanged: {
            if (visible)
                pointTextField.forceActiveFocus()
        }

/*
        onEditingFinished: {
            if(saveButton.canSave)
                saveButton.released()
        }
*/
        background: Rectangle {
            radius: 2
            border.color: nameError ? Style.errorColor : pointTextField.activeFocus ? Style.lightBlue : Style.lightGreyBorder
            border.width: pointTextField.activeFocus || nameError ? 3 : 1
        }

        onTextChanged: checkPoint(Helper.formatName(pointTextField.text), oldName,
                       tmpPointView.x,
                       tmpPointView.y)
    }

    Label {
        id: groupLabel
        text: langue == "English" ? qsTr("Choose Group") : qsTr("选择分组")
        font.pointSize: Style.ubuntuSubHeadingSize
        color: Style.midGrey2
        anchors {
            left: parent.left
            top: pointTextField.bottom
            right: parent.right
            topMargin: 20
        }
    }

    CustomComboBox {
        id: groupComboBox
        model: pointModel
        font.pointSize: Style.ubuntuSubHeadingSize
//        displayText: {
//            if (oldname) {
//                oldGroup
//            } else {
//                langue == "English" ? Helper.noGroupChinese : Helper.noGroup
//            }
//        }
        langue: createPointMenuFrame.langue
        anchors {
            left: parent.left
            top: groupLabel.bottom
            right: parent.right
            topMargin: 8
        }
    }

//    Label {
//            id: groupRobotLabel
//            text: langue == "English" ? qsTr("选择分组") : qsTr("Choose Robot")
//            color: Style.midGrey2
//            anchors {
//                left: parent.left
//                top: groupComboBox.bottom
//                right: parent.right
//                topMargin: 20
//            }
//        }

//        CustomComboBoxRobot {
//            id: groupRobotComboBox
//            model: robotModel
//            displayText: langue == "English" ? Helper.noRobotChinese : Helper.noRobot
//            anchors {
//                left: parent.left
//                top: groupRobotLabel.bottom
//                right: parent.right
//                topMargin: 8
//            }
//        }

    Label {
        id: homeLabel
        text: langue == "English" ? qsTr("Charging Station") : qsTr("充电站")
        font.pointSize: Style.ubuntuSubHeadingSize
        color: Style.midGrey2
        anchors {
            left: parent.left
            top: groupComboBox.bottom
            right: parent.right
            topMargin: 20
        }
    }

    SquareCheckBox {
        id: homeCheckBox
        width:10
        focusPolicy: Qt.WheelFocus
        anchors {
            left: parent.left
            top: homeLabel.bottom
            topMargin: 12
           // right: parent.right
        }



        onCheckedChanged: tmpPointView.setType(homeCheckBox.checked ? Helper.PointViewType.HOME_TEMP : Helper.PointViewType.TEMP);
    }
    CustomLabel{
        text: langue == "English" ? "Set to charging station" : "设置为充电站"
         anchors.verticalCenterOffset: 1
         anchors.leftMargin: 15
         font.pointSize: Style.ubuntuSubTextSize
         color:Style.midGrey2
         anchors{
             left: homeCheckBox.right
             verticalCenter: homeCheckBox.verticalCenter
         }
    }

    Label {
        id: oriLabel
        text: langue == "English" ? qsTr("Orientation") : qsTr("方向")
        font.pointSize: Style.ubuntuTextSize
        color: Style.midGrey2
        anchors {
            left: parent.left
            top: homeCheckBox.bottom
            topMargin: 20
        }
        width: 80
    }

    TextField {
        id: field

        background: Rectangle {
            border.color: field.activeFocus ? Style.lightBlue : Style.lightGreyBorder
            border.width: 2
        }

        font.pointSize: Style.ubuntuSubHeadingSize
        width: 40
        height: 21
//        visible: homeCheckBox.checked

        padding: 0
        selectByMouse: true
        // range of accepted values : 0 to 359
        validator: IntValidator { bottom: 0; top: 359 }
        inputMethodHints: Qt.ImhDigitsOnly
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignLeft
        placeholderText: "0"
        color: Style.darkSkyBlue
        anchors {
            left: oriLabel.right
            leftMargin: 15
            verticalCenter: oriLabel.verticalCenter
        }
        // to update the slider value accordingly
        onAccepted: {
            focus = false;
            slider.value = parseInt(text);
        }
    }

    Button {
        id: incButton

        width: 12
        height: 21
//        visible: homeCheckBox.checked

        anchors {
            left: field.right
            leftMargin: 8
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
                                slider.value = slider.value - 1
                            else
                                slider.value = slider.value + 1
                        }
                    }
                }

                onPressed: timer.start()

                onReleased: timer.stop()
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
            left: parent.left
            top: oriLabel.bottom
            right: parent.right
            leftMargin: 10
            topMargin: 10
            rightMargin: 10
        }
        onPositionChanged: {
            tmpPointView.setOrientation(Math.round(slider.valueAt(slider.position)));
            field.text = Math.round(valueAt(position))
        }
    }


    Label {
        id: pointLocationLabel
        text: langue == "English" ? qsTr("Point Location") : qsTr("P目标点位置")
        font.pointSize: Style.ubuntuSubTextSize
        color: Style.midGrey2
        anchors {
            left: parent.left
            top: slider.bottom
            right: parent.right
            topMargin: 20
        }
    }

    Label {
        id: xLabel
        text: qsTr("X : " + Math.round(tmpPointView.x))
        font.pointSize: Style.ubuntuTextSize
        anchors {
            left: parent.left
            top: pointLocationLabel.bottom
            right: parent.right
            topMargin: 8
        }
    }

    Label {
        text: qsTr("Y : " + Math.round(tmpPointView.y))
        font.pointSize: Style.ubuntuTextSize
        anchors {
            left: parent.left
            top: xLabel.bottom
            right: parent.right
        }
    }

    CancelButton {
        langue: createPointMenuFrame.langue
        anchors {
            left: parent.left
            right: parent.horizontalCenter
            bottom: parent.bottom
            rightMargin: 5
        }
        onClicked: backToMenu()
    }

    SaveButton {
        id: saveButton
        langue: createPointMenuFrame.langue
        anchors {
            left: parent.horizontalCenter
            right: parent.right
            bottom: parent.bottom
            leftMargin: 5
        }
        canSave: false
        tooltip: errorMsg
        onReleased: if(saveButton.canSave) {
            var newName = Helper.formatName(pointTextField.text);
            var groupName = groupComboBox.displayText;
            if (groupComboBox.displayText === Helper.noGroupChinese) {
                groupComboBox.displayText = Helper.noGroup;
            }
//            var action = 1; // feature with robotGroup

//            var groupRobotName = groupRobotComboBox.displayText;
//            var finalGroupName = ""

//            if (groupRobotName !== Helper.noRobot && groupRobotName !== Helper.noRobotChinese) {
//                finalGroupName = groupRobotName;
//            } else {
//                finalGroupName = groupComboBox.displayText;
//            }

            createPoint(newName,  groupComboBox.displayText, tmpPointView.x, tmpPointView.y,
                        oldName, oldGroup, true, homeCheckBox.checked,
                        Math.round(slider.valueAt(slider.position)));

            homeName = ""; /// reset homeName

//            createPoint(newName, groupComboBox.displayText, tmpPointView.x, tmpPointView.y,
//                        oldName, oldGroup, true, homeCheckBox.checked,
//                        Math.round(slider.valueAt(slider.position)));
            backToMenu();
            var mess1 = ''
            var mess2 = ''
            if (langue == "English") {
                 mess1 = "Created the point \"" + newName + "\" in \"" + groupName + "\""
//                mess2 = "编辑目标点 \"" + oldName + "\" 从 \"" + oldGroup + "\" 到 \"" + newName + "\" 到 \"" + groupName + "\""
            } else {
                 mess1 = "已创建目标点 \"" + newName + "\" 在 \"" + groupName + "\""
//                mess2 = "Edited the point \"" + oldName + "\" from \"" + oldGroup + "\" to \"" + newName + "\" in \"" + groupName + "\""
            }

//            setMessageTop(3, oldName === "" ? mess1 : mess2)
            setMessageTop(3, mess1);
        }
    }

    function enableSave(posError, _nameError){
        nameError = _nameError;
        saveButton.canSave = !posError && !nameError;

        errorMsg = "";
        var mess1 = ''
        var mess2 = ''
        var mess3 = ''
        if (langue == "English") {
            mess1 = "You cannot save this point because your robot(s) would not be able to go there"
            mess2 = "The point name cannot be empty"
            mess3 = "The point name \"" + Helper.formatName(pointTextField.text) + "\" is already taken"
        } else {
            mess1 = "无法保存目标点，因为机器人无法去到那里"
            mess2 = "目标点名称不能为空"
            mess3 = "目标点名 \"" + Helper.formatName(pointTextField.text) + "\" 已经被使用"
        }

        if(!saveButton.canSave){
            if(Helper.formatName(pointTextField.text) === "")
                errorMsg = mess2;
            else if(nameError)
                errorMsg = mess3;

            if(posError)
                errorMsg += (nameError ? "\n" : "") + mess1;
        }
        setMessageTop(1, errorMsg);
    }
}

