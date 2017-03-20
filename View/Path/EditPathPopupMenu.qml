import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Path"

Menu {
    id: menu
    padding: 0
    width: 188

    property Paths pathModel
    property string myGroup

    signal editPath()
    signal deletePath()
    signal moveTo(string newGroup)

    background: Rectangle {
        implicitWidth: parent.width
        implicitHeight: 3 * Style.menuItemHeight + 4
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    MenuItem {
        text: qsTr("Edit Path")
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: editPath()
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 2
    }

    MenuItem {
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
                implicitHeight: pathModel.count * Style.menuItemHeight + 2
                color: Style.lightGreyBackground
                border.color: Style.lightGreyBorder
                radius: 5
            }

            MenuItem {
                height: Style.menuItemHeight
                text: qsTr(Helper.noGroup)
                width: parent.width
                leftPadding: Style.menuItemLeftPadding
                /// Disable the group in which the path already is so we can't move it in
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
                    model: pathModel

                    MenuItem {
                        anchors {
                            left: parent.left
                            right: parent.right
                        }
                        visible: groupName !== Helper.noGroup
                        Layout.preferredHeight: visible ? Style.menuItemHeight : 0
                        leftPadding: Style.menuItemLeftPadding
                        /// Disable the group in which the path already is so we can't move it in
                        enabled: !(groupName === myGroup)

                        Label {
                            text: qsTr(groupName)
                            anchors {
                                left: parent.left
                                right: parent.right
                                leftMargin: 20
                                rightMargin: 5
                                verticalCenter: parent.verticalCenter
                            }
                            maximumLineCount: 1
                            elide: Text.ElideRight
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

    MenuItem {
        text: qsTr("Delete Path")
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: deletePath()
    }
}
