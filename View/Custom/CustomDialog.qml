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
    property string colorBackground: "#f3f3f3"
    property string colorBorder: "#bcb5b9"
    property string textColor: Style.greyText
    property int bottomMarginLabel: 10
    property int leftMarginLabel: 10
    property int rightMarginLabel: 10
    property int topMarginLabel: 10

    signal yes()

    background: Rectangle {
//        color: "#f3f3f3"
        color: colorBackground
        border.width: 2
        border.color: colorBorder
        radius: 5
    }

    height: 150
    width: 400

    header: Label {
        id: customHeader
        background: Rectangle {
//            color: colorBackground

            color: Style.warningHeaderColor
            width: parent.width - 5
            radius: 3
        }
        anchors {
            top: parent.top
            left: parent.left
            topMargin: 2
            leftMargin: 2
            rightMargin: 1

        }

       //font.bold: true
        text:qsTr(title)

        color: (title === "WARNING" || title === "警告") ? "white" : "white"
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
//                bottomMargin: 10
                bottomMargin: bottomMarginLabel
                topMargin: customHeader.height + topMarginLabel
//                leftMargin: 10
                leftMargin: leftMarginLabel
                rightMargin: rightMarginLabel
            }
            text: qsTr(message)
            wrapMode: Text.WordWrap
            color: textColor
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
