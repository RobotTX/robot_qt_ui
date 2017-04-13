import QtQuick 2.7
//import QtQuick.Controls 1.4
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../Custom"

Frame {
    id: topViewFrame
    property bool hasMap
    signal savePosition()
    signal loadPosition()
    padding: 0

    Layout.minimumHeight: Style.menuHeaderHeight
    Layout.maximumHeight: Math.max(Style.menuHeaderHeight, flick.contentItem.childrenRect.height + 15)

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    Flickable {
        id: flick
        ScrollBar.vertical: ScrollBar { }
        contentHeight: contentItem.childrenRect.height
        clip: true
        anchors {
            left: parent.left
            top: parent.top
            right: saveStateButton.left
            bottom: parent.bottom
            leftMargin: 10
            topMargin: 5
            bottomMargin: 5
        }

        MouseArea {
            onWheel: console.log
        }

        Column {
            anchors.left: parent.left
            anchors.right: parent.right
            Label {
                id: errorLabel
                color: Style.errorColor2
                visible: text !== ""
                text: qsTr("Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here ")
                wrapMode: Text.WordWrap
                width: parent.width
            }
            Label {
                id: warningLabel
                color: Style.warningColor
                visible: text !== ""
                text: qsTr("Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here ")
                wrapMode: Text.WordWrap
                width: parent.width
            }
            Label {
                id: successLabel
                color: Style.successColor
                visible: text !== ""
                text: qsTr("Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here ")
                wrapMode: Text.WordWrap
                width: parent.width
            }
            Label {
                id: infoLabel
                color: Style.infoColor
                visible: text !== ""
                text: qsTr("Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here Soon some awesome messages here ")
                wrapMode: Text.WordWrap
                width: parent.width
            }
        }
    }

    /// The load state button
    SmallButton {
        id: loadStateButton
        imgSrc: "qrc:/icons/loadState"
        anchors {
            top: parent.top
            topMargin: 10
            right: parent.right
            rightMargin: 10
        }
        enabled: hasMap
        onClicked: topViewFrame.loadPosition()
    }

    /// The save state button
    SmallButton {
        id: saveStateButton
        imgSrc: "qrc:/icons/saveState"
        anchors {
            top: parent.top
            topMargin: 10
            right: loadStateButton.left
            rightMargin: 14
        }
        enabled: hasMap
        onClicked: topViewFrame.savePosition()
    }

    /// TODO add Timer ?
    function setMessageTop(label, msg){
        switch(label){
            case 0:
                errorLabel.text = msg;
            break;
            case 1:
                warningLabel.text = msg;
            break;
            case 2:
                successLabel.text = msg;
            break;
            case 3:
                infoLabel.text = msg;
            break;
            default:
                console.log("Not supposed to be here");
            break;
        }
    }
}
