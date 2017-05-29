import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../Custom"

Dialog {
    id: dialog
    modal: true
    property string ip
    property string message
    property string rejectMessage
    property string acceptMessage
    property string yesMessage
    signal yes()

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
        text: qsTr(title)
    }

    contentItem: Rectangle {
        anchors.fill: parent
        color: "transparent"

        MouseArea {
            width: dialog.parent.width
            height: dialog.parent.height
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
        }

        Label {
            anchors {
                top: parent.top
                left: parent.left
                right: parent.right
                bottom: layout.top
                topMargin: customHeader.height + 10
                leftMargin: 10
                rightMargin: 10
            }
            text: qsTr(message)
            wrapMode: Text.WordWrap
        }

        RowLayout {
            id: layout
            anchors {
                left: parent.left
                right: parent.right
                bottom: parent.bottom
                bottomMargin: 10
            }

            CancelButton {
                id: robotButton
                visible: rejectMessage !== ""
                txt: qsTr(rejectMessage)
                onClicked: dialog.reject()
                Layout.preferredHeight: height
                Layout.fillWidth: true
                Layout.leftMargin: 10
                Layout.rightMargin: 10
            }

            SaveButton {
                id: yesButton
                visible: yesMessage !== ""
                txt: yesMessage
                onReleased: dialog.yes()
                Layout.preferredHeight: height
                Layout.fillWidth: true
                Layout.leftMargin: 10
                Layout.rightMargin: 10
            }

            SaveButton {
                id: appButton
                visible: acceptMessage !== ""
                txt: acceptMessage
                onReleased: dialog.accept()
                Layout.preferredHeight: height
                Layout.fillWidth: true
                Layout.leftMargin: 10
                Layout.rightMargin: 10
            }
        }
    }
}
