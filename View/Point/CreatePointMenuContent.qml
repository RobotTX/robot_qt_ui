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
    property string errorMsg
    property double oldPosX
    property double oldPosY

    signal backToMenu()
    signal createPoint(string name, string groupName, double x, double y, string oldName, string oldGroup)
    signal checkPoint(string name, string oldName, double x, double y)
    signal setMessageTop(int status, string msg)

    Connections {
        target: tmpPointView
        onTmpPointViewPosChanged: {
            checkPoint(Helper.formatName(pointTextField.text), oldName,
                       tmpPointView.x + tmpPointView.width / 2,
                       tmpPointView.y + tmpPointView.height)
        }
    }

    onVisibleChanged: {
        if(tmpPointView && createPointMenuFrame)
            tmpPointView._isVisible = visible;

        if(!visible){
            if(tmpPointView && createPointMenuFrame){
                tmpPointView.setPos(tmpPointView.originX, tmpPointView.originY);
            }
            oldName = "";
            oldGroup = "";
            oldPosX = 0;
            oldPosY = 0;
            groupComboBox.currentIndex = 0;
            groupComboBox.displayText = Helper.noGroup;
        } else {
            if(oldName !== ""){
                for(var i = 0; i < pointModel.count; i++)
                    if(pointModel.get(i).groupName === oldGroup)
                        for(var j = 0; j < pointModel.get(i).points.count; j++)
                            if(pointModel.get(i).points.get(j).name === oldName){
                                pointModel.get(i).points.setProperty(j, "isVisible", false);

                                tmpPointView.setPos(pointModel.get(i).points.get(j).posX, pointModel.get(i).points.get(j).posY);
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
            right: parent.right
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

        onEditingFinished: {
            if(saveButton.canSave)
                saveButton.clicked()
        }

        background: Rectangle {
            radius: 2
            border.color: nameError ? Style.errorColor : pointTextField.activeFocus ? Style.lightBlue : Style.lightGreyBorder
            border.width: pointTextField.activeFocus || nameError ? 3 : 1
        }
        onTextChanged: checkPoint(Helper.formatName(pointTextField.text), oldName,
                       tmpPointView.x + tmpPointView.width / 2,
                       tmpPointView.y + tmpPointView.height)
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
        id: pointLocationLabel
        text: qsTr("Point Location")
        color: Style.midGrey2
        anchors {
            left: parent.left
            top: groupComboBox.bottom
            right: parent.right
            topMargin: 20
        }
    }

    Label {
        id: xLabel
        text: qsTr("X : " + Math.round(tmpPointView.x + tmpPointView.width / 2))
        anchors {
            left: parent.left
            top: pointLocationLabel.bottom
            right: parent.right
            topMargin: 8
        }
    }

    Label {
        text: qsTr("Y : " + Math.round(tmpPointView.y + tmpPointView.height))
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
            createPoint(newName, groupName, tmpPointView.x + tmpPointView.width / 2, tmpPointView.y + tmpPointView.height, oldName, oldGroup);
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
                errorMsg = "The point name can not be empty";
            else if(nameError)
                errorMsg = "The point name \"" + Helper.formatName(pointTextField.text) + "\" is already taken";

            if(posError)
                errorMsg += (nameError ? "\n" : "") + "You cannot save this point because your robot(s) would not be able to go there";
        }
        setMessageTop(1, errorMsg);
    }
}
