import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Model/Point"
import "../Custom"

Frame {
    id: createGroupMenuFrame
    objectName: "createGroupMenuFrame"

    signal backToMenu()
    signal createGroup(string name)
    signal checkGroup(string name)

    onVisibleChanged: groupTextField.text = "";

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
            createGroup(groupTextField.text);
            backToMenu();
        }
    }

    function enableSave(enable){
        saveButton.enabled = enable;
    }
}
