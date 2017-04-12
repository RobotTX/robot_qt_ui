import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../Custom"

Menu {
    id: menu
    padding: 0
    width: 188

    property Points pointModel
    property string myGroup

    signal editPoint()
    signal deletePoint()
    signal moveTo(string newGroup)

    background: Rectangle {
        implicitWidth: parent.width
        implicitHeight: 3 * Style.menuItemHeight + 4
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    PopupMenuItem {
        text: qsTr("Edit Point")
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: editPoint()
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 2
    }

    PopupMenuItem {
        text: qsTr("Move to")
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight

        Image {
            asynchronous: true
            source: "qrc:/icons/arrow"
            fillMode: Image.Pad // For not stretching image
            anchors.verticalCenter: parent.verticalCenter
            anchors.right: parent.right
            anchors.rightMargin: 12
        }
        onHoveredChanged: if(visible && !moveToMenu.visible) moveToMenu.open()


        Menu {
            id: moveToMenu
            padding: 0
            width: 140
            x: parent.width

            background: Rectangle {
                implicitWidth: parent.width
                implicitHeight: pointModel.count * Style.menuItemHeight + 2
                color: Style.lightGreyBackground
                border.color: Style.lightGreyBorder
                radius: 5
            }

            PopupMenuItem {
                height: Style.menuItemHeight
                text: qsTr(Helper.noGroup)
                width: parent.width
                leftPadding: Style.menuItemLeftPadding
                /// Disable the group in which the point already is so we can't move it in
                enabled: !(Helper.noGroup === myGroup)
                onTriggered: moveTo(Helper.noGroup)
            }

            Rectangle {
                color: Style.lightGreyBorder
                width: moveToMenu.width
                height: 2
            }

            ColumnLayout {
                anchors {
                    left: parent.left
                    right: parent.right
                }

                Repeater {
                    model: pointModel

                    PopupMenuItem {
                        anchors {
                            left: parent.left
                            right: parent.right
                        }
                        visible: groupName !== Helper.noGroup
                        Layout.preferredHeight: visible ? Style.menuItemHeight : 0
                        leftPadding: Style.menuItemLeftPadding
                        /// Disable the group in which the point already is so we can't move it in
                        enabled: !(groupName === myGroup)

                        CustomLabel {
                            text: qsTr(groupName)
                            anchors {
                                left: parent.left
                                right: parent.right
                                leftMargin: 20
                                rightMargin: 5
                                verticalCenter: parent.verticalCenter
                            }
                            color: enabled ? "black" : "lightgrey"
                            enabled: !(groupName === myGroup)
                        }

                        onTriggered: moveTo(groupName)
                    }
                }
            }
        }
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 2
    }

    PopupMenuItem {
        text: qsTr("Delete Point")
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: deletePoint()
    }
}
