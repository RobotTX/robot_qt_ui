import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Model/Robot"
import "../Custom"
import "../../Helper/style.js" as Style

Frame {

    id: leftMenu

    padding: 0
    width: 180

    background: Rectangle {
        anchors.fill: parent
        border.color: Style.lightGreyBorder
        color: Style.lightGreyBackground
    }

    property ListModel robotsList

    Flickable {

        anchors {
            top: parent.top
            bottom: saveButton.top
            left: parent.left
            topMargin: 14
            leftMargin: 14
            bottomMargin: 14
            rightMargin: 4
            right: parent.right
        }

        clip: true

        ScrollBar.vertical: ScrollBar {  }

        contentHeight: contentItem.childrenRect.height

        ColumnLayout {

            spacing: 30

            anchors {
                left: parent.left
                right: parent.right
                top: parent.top
            }

            Repeater {

                model: robotsList

                MergeMapMenuItem {
                    onRemoveMap: {
                        console.log("removing ip" + ip)
                        robotsList.removeRobot(ip)
                    }
                }
            }
        }
    }

    CancelButton {
        id: cancelButton
        anchors {
            bottom: parent.bottom
            bottomMargin: 17
            left: parent.left
            leftMargin: 18
            right: parent.right
            rightMargin: 18
        }
        width: 70
    }

    SaveButton {
        id: saveButton
        anchors {
            bottom: cancelButton.top
            bottomMargin: 11
            left: parent.left
            leftMargin: 18
            right: parent.right
            rightMargin: 18
        }
        width: cancelButton.width
    }
}
