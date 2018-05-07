import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../Custom"

Frame {
    // used to create a group (points or paths)
    id: createGroupMenuContentFrame

    property string oldName
    property string errorMsg
    property string langue

    signal backToMenu()
    signal createGroup(string name)
    signal renameGroup(string newName, string oldName)
    signal checkGroup(string name)
    signal setMessageTop(int status, string msg)


    onVisibleChanged: {
        groupTextField.text = oldName;
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
        id: groupLabel
        text: langue == "English" ? qsTr("组名称") : qsTr("Group Name")
        color: Style.midGrey2
        anchors {
            left: parent.left
            top: parent.top
            right: parent.right
        }
    }

    TextField {
        id: groupTextField
        selectByMouse: true
        placeholderText: langue == "English" ? qsTr("组名称") : qsTr("Enter name")
        height: 28
        anchors {
            left: parent.left
            top: groupLabel.bottom
            right: parent.right
        }
        anchors.topMargin: 8
/*
        onEditingFinished: {
            if(saveButton.canSave)
                saveButton.released()
        }
*/
        background: Rectangle {
            radius: 2
            border.color: !saveButton.canSave ? Style.errorColor : groupTextField.activeFocus ? Style.lightBlue : Style.lightGreyBorder
            border.width: groupTextField.activeFocus || !saveButton.canSave ? 3 : 1
        }
        onTextChanged: {
            console.log(text)
            if(oldName === groupTextField.text)
                saveButton.canSave = true
            else
                checkGroup(Helper.formatName(groupTextField.text))
        }
    }

    CancelButton {
        langue: createGroupMenuContentFrame.langue
        anchors {
            left: parent.left
            right: parent.horizontalCenter
            bottom: parent.bottom
        }
        anchors.rightMargin: 5
        onClicked: backToMenu()
    }

    SaveButton {
        id: saveButton
        langue: createGroupMenuContentFrame.langue
        anchors {
            left: parent.horizontalCenter
            right: parent.right
            bottom: parent.bottom
        }
        canSave: false
        tooltip: errorMsg
        anchors.leftMargin: 5
        onReleased: if(saveButton.canSave) {
            var newName = Helper.formatName(groupTextField.text);
            if(oldName === ""){
                createGroup(newName);
                setMessageTop(3, "Created the group \"" + newName + "\"");
            } else {
                renameGroup(newName, oldName);
                setMessageTop(3, "Renamed the group \"" + oldName + "\" to \"" + newName + "\"");
            }
            console.log("creating path " + newName)
            backToMenu();
        }
    }

    function enableSave(enable){
        saveButton.canSave = enable;
        errorMsg = enable ? "" : Helper.formatName(groupTextField.text) === "" ? "The name of the group cannot be empty" : "\"" + groupTextField.text + "\" is already taken";
        setMessageTop(1, errorMsg);
    }
}
