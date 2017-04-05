import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../Custom"

Dialog {
    id: dialog
    modal: true
    property string ip
    property string message
    property string rejectMessage
    property string acceptMessage

    background: Rectangle {
        color: "#f3f3f3"
        border.width: 2
        border.color: "#bcb5b9"
        radius: 5
    }

    height: 150
    width: 400

    header: Label {
        id: customHeader
        background: Rectangle {
            color: "transparent"
        }
        anchors {
            top: parent.top
            left: parent.left
            topMargin: 5
            leftMargin: 10
        }

        font.bold: true
        text: title
    }

    contentItem: Rectangle {
        anchors.fill: parent
        color: "transparent"

        Label {
            anchors {
                top: parent.top
                left: parent.left
                right: parent.right
                bottom: robotButton.top
                topMargin: customHeader.height + 10
                leftMargin: 10
                rightMargin: 10
            }
            text: qsTr(message)
            wrapMode: Text.WordWrap
        }

        CancelButton {
            id: robotButton
            anchors {
                left: parent.left
                right: parent.horizontalCenter
                bottom: parent.bottom
                leftMargin: 10
                rightMargin: 10
                bottomMargin: 10
            }
            txt: rejectMessage
            onClicked: dialog.reject()
        }

        SaveButton {
            id: appButton
            txt: acceptMessage
            anchors {
                left: parent.horizontalCenter
                right: parent.right
                bottom: parent.bottom
                leftMargin: 10
                rightMargin: 10
                bottomMargin: 10
            }
            onClicked: dialog.accept()
        }
    }
}
