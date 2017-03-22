import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Model/Point"
import "../Custom"

Frame {
    property string oldName: ""

    signal backToMenu()
    signal createGroup(string name)
    signal renameGroup(string newName, string oldName)
    signal checkGroup(string name)

    onVisibleChanged: groupTextField.text = oldName;

    padding: 20

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    Label {
        id: groupLabel
        text: qsTr("Group Name")
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
        placeholderText: qsTr("Enter name")
        height: 28
        anchors {
            left: parent.left
            top: groupLabel.bottom
            right: parent.right
        }
        anchors.topMargin: 8

        background: Rectangle {
            radius: 2
            border.color: groupTextField.activeFocus ? Style.lightBlue : Style.lightGreyBorder
            border.width: groupTextField.activeFocus ? 3 : 1
        }
        onTextChanged: checkGroup(groupTextField.text)
    }

    CancelButton {
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
        anchors {
            left: parent.horizontalCenter
            right: parent.right
            bottom: parent.bottom
        }
        enabled: false
        anchors.leftMargin: 5
        onClicked: {
            if(oldName === "")
                createGroup(groupTextField.text);
            else
                renameGroup(groupTextField.text, oldName);
            backToMenu();
        }
    }

    function enableSave(enable){
        saveButton.enabled = enable;
    }
}
