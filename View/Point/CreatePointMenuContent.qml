import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Controls.Styles 1.4
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../Custom"

Frame {
    id: createPointMenuFrame
    objectName: "createPointMenuFrame"
    property Points pointModel
    property PointView tmpPointView
    property string oldName: ""
    property string oldGroup
    property bool nameError: true
    property bool wasDisplayed: false
    property string errorMsg

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
        if(tmpPointView && createPointMenuFrame){
            tmpPointView._isVisible = visible;
            tmpPointView.x =0;
            tmpPointView.y = 0;
        }

        homeCheckBox.checked = false;
        slider.value = 0;

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
            groupComboBox.currentIndex = 0;
            groupComboBox.displayText = Helper.noGroup;
        } else {
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
        text: qsTr("Point Name")
        color: Style.midGrey2
        anchors {
            left: parent.left
            top: parent.top
        }
    }

    TextField {
        id: pointTextField
        selectByMouse: true
        placeholderText: qsTr("Enter name")

        text: oldName
        height: 28
        anchors {
            left: parent.left
            top: pointLabel.bottom
            right: parent.right
            topMargin: 8
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
        text: qsTr("Choose Group")
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
        displayText: oldName ? oldGroup : Helper.noGroup
        anchors {
            left: parent.left
            top: groupLabel.bottom
            right: parent.right
            topMargin: 8
        }
    }

    Label {
        id: homeLabel
        text: qsTr("Charging station")
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
        anchors {
            left: parent.left
            top: homeLabel.bottom
            right: parent.right
            topMargin: 10
        }
        text: "This point is a charging station"
        onCheckedChanged: tmpPointView.setType(homeCheckBox.checked ? Helper.PointViewType.HOME_TEMP : Helper.PointViewType.TEMP);
    }

    Label {
        id: oriLabel
//        visible: homeCheckBox.checked
        text: qsTr("Orientation")
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
        font.pointSize: 10
        anchors {
            left: oriLabel.right
            leftMargin: 4
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
        text: qsTr("Point Location")
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
        anchors {
            left: parent.left
            top: pointLocationLabel.bottom
            right: parent.right
            topMargin: 8
        }
    }

    Label {
        text: qsTr("Y : " + Math.round(tmpPointView.y))
        anchors {
            left: parent.left
            top: xLabel.bottom
            right: parent.right
        }
    }

    CancelButton {
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
            createPoint(newName, groupName, tmpPointView.x, tmpPointView.y,
                        oldName, oldGroup, true, homeCheckBox.checked,
                        Math.round(slider.valueAt(slider.position)));
            backToMenu();
            setMessageTop(2, oldName === "" ? "Created the point \"" + newName + "\" in \"" + groupName + "\"" :
                                            "Edited a point from \"" + oldName + "\" in \"" + oldGroup + "\" to \"" + newName + "\" in \"" + groupName + "\"")
        }
    }

    function enableSave(posError, _nameError){
        nameError = _nameError;
        saveButton.canSave = !posError && !nameError;

        errorMsg = "";
        if(!saveButton.canSave){
            if(Helper.formatName(pointTextField.text) === "")
                errorMsg = "The point name cannot be empty";
            else if(nameError)
                errorMsg = "The point name \"" + Helper.formatName(pointTextField.text) + "\" is already taken";

            if(posError)
                errorMsg += (nameError ? "\n" : "") + "You cannot save this point because your robot(s) would not be able to go there";
        }
        setMessageTop(1, errorMsg);
    }
}
