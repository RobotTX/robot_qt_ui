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
        if (groupTextField.text === "") {
            saveButton.canSave = false;
        }

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
        placeholderText: langue == "English" ? qsTr("输入组名称") : qsTr("Enter name")
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

        tooltip: errorMsg
        canSave: false
        anchors.leftMargin: 5
        onReleased: if(saveButton.canSave) {
            var newName = Helper.formatName(groupTextField.text);
            var creationGroup = "";
            var renameGroupLabel = "";
            if (langue === "English") {
                creationGroup = "已创建分组";
                renameGroupLabel = "已重命名分组 \"" +  oldName + "\" 为 " + "\"" + newName + "\"";
            } else {
                creationGroup = "Created the group \"";
                renameGroupLabel = "Renamed the group \"" + oldName + "\" to \"" + newName + "\"";
            }

            if(oldName === ""){
                console.log("oldName === empty");
                createGroup(newName);
                setMessageTop(3, creationGroup + newName + "\"");
            } else {
                console.log("oldName !== empty");
                renameGroup(newName, oldName);
                setMessageTop(3, renameGroupLabel);
            }
            backToMenu();
        }
    }

    function enableSave(enable){
        saveButton.canSave = enable;
        var msg1 = "";
        var msg2 = "";
        if (langue === "English") {
            msg1 = "组名称不可以为空";
            msg2 = "\"" + groupTextField.text + "\" 已经被占用";
        } else {
            msg1 = "The name of the group cannot be empty";
            msg2 = "\"" + groupTextField.text + "\" is already taken";
        }

        errorMsg = enable ? "" : Helper.formatName(groupTextField.text) === "" ? msg1 : msg2;
        setMessageTop(1, errorMsg);
    }
}
