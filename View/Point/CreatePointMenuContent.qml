import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Controls.Styles 1.4
import "../../Helper/style.js" as Style
import "../../Model/Point"
import "../Custom"

Frame {
    objectName: "createPointMenuFrame"
    property Points pointModel
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
        placeholderText: qsTr("Enter name")
        height: 28
        anchors {
            left: parent.left
            top: pointLabel.bottom
            right: parent.right
        }
        anchors.topMargin: 8

        background: Rectangle {
            radius: pointTextField.activeFocus ? 2 : 0
            border.color: pointTextField.activeFocus ? Style.lightBlue : Style.lightGreyBorder
            border.width: pointTextField.activeFocus ? 3 : 1
        }
        // a9d1fc
    }

    Label {
        id: groupLabel
        text: qsTr("Choose Group")
        color: Style.midGrey2
        anchors {
            left: parent.left
            top: pointTextField.bottom
            right: parent.right
        }
        anchors.topMargin: 20
    }

    CustomComboBox {
        id: groupComboBox
        model: getGroupList()
    }

    Label {
        id: pointLocationLabel
        text: qsTr("Point Location")
        color: Style.midGrey2
        anchors {
            left: parent.left
            top: groupComboBox.bottom
            right: parent.right
        }
        anchors.topMargin: 20
    }

    Label {
        id: xLabel
        text: qsTr("X : 147.1")
        anchors {
            left: parent.left
            top: pointLocationLabel.bottom
            right: parent.right
        }
        anchors.topMargin: 8
    }

    Label {
        text: qsTr("Y : 132.8")
        anchors {
            left: parent.left
            top: xLabel.bottom
            right: parent.right
        }
    }

    function getGroupList(){
        var groups = [];
        var lastGroup = "";
        for(var i = 0; i < pointModel.count; i++){
            var groupName = pointModel.get(i)._groupName;
            if(groupName !== lastGroup && groupName !== ""){
                groups.push(groupName);
                lastGroup = groupName;
            }
        }

        return groups;
    }
}
