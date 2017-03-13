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

    Connections {
        target: tmpPointView
        onTmpPointViewPosChanged: {
            checkPoint(pointTextField.text,
                       tmpPointView.x + tmpPointView.width / 2,
                       tmpPointView.y + tmpPointView.height)
        }
    }

    signal backToMenu()
    signal createPoint(string name, string groupName, double x, double y)
    signal checkPoint(string name, double x, double y)

    onVisibleChanged: {
        if(tmpPointView && createPointMenuFrame){
            tmpPointView.isVisible = createPointMenuFrame.visible;
            tmpPointView.x = tmpPointView.originX;
            tmpPointView.y = tmpPointView.originY;
        }
        pointTextField.text = "";
        groupComboBox.currentIndex = 0;
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
        placeholderText: qsTr("Enter name")
        height: 28
        anchors {
            left: parent.left
            top: pointLabel.bottom
            right: parent.right
        }
        anchors.topMargin: 8

        background: Rectangle {
            radius: 2
            border.color: pointTextField.activeFocus ? Style.lightBlue : Style.lightGreyBorder
            border.width: pointTextField.activeFocus ? 3 : 1
        }
        onTextChanged: checkPoint(pointTextField.text,
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
        }
        anchors.topMargin: 20
    }

    CustomComboBox {
        id: groupComboBox
        model: Helper.getGroupList(pointModel)
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
        text: qsTr("X : " + Math.round(tmpPointView.x + tmpPointView.width / 2))
        anchors {
            left: parent.left
            top: pointLocationLabel.bottom
            right: parent.right
        }
        anchors.topMargin: 8
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
            createPoint(pointTextField.text, groupComboBox.currentText, tmpPointView.x + tmpPointView.width / 2, tmpPointView.y + tmpPointView.height);
            backToMenu();
        }
    }

    function enableSave(enable){
        saveButton.enabled = enable;
    }
}
