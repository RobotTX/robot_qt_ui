import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Path"
import "../../Model/Robot"
import "../../Model/Point"
import "../Custom"
import "../Robot"

Menu {
    id: menu
    padding: 0
    width: 188

    property Paths pathModel
    property Robots robotModel
    property Points pointModel
    property string myGroup
    property string myRobot
    property int currentMenuIndex: -1

    signal editPath()
    signal deletePath()
    signal moveTo(string newGroup)
    signal robotSelected(string robot)

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    // first item
    PopupMenuItem {
        labelText: "Edit Path"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: editPath()
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    PopupMenuItem {
         labelText: "Test"
         width: parent.width
         leftPadding: Style.menuItemLeftPadding
         height: Style.menuItemHeight

         Image {
             asynchronous: true
             source: "qrc:/icons/arrow"
             fillMode: Image.Pad // for not streaching image
             anchors.verticalCenter: parent.verticalCenter
             anchors.right: parent.right
             anchors.rightMargin: 12
         }

         onHoveredChanged: if(!robotListInPopup.visible) robotListInPopup.open()

         RobotListInPopup {
             id: robotListInPopup
//             onVisibleChanged: robotListInPopup.close()
             x: parent.width
             robotModel: menu.robotModel
             onRobotSelected: {
                 console.log("Robot selected !");
             }
         }
    }


    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    // third item
    PopupMenuItem {
        labelText: "Move to"
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
        onHoveredChanged: if(!moveToMenu.visible) moveToMenu.open()

        Menu {
            id: moveToMenu
            padding: 0
//            onVisibleChanged: moveToMenu.close()
            width: 140
            x: parent.width

            background: Rectangle {
                color: Style.lightGreyBackground
                border.color: Style.lightGreyBorder
                radius: 5
            }

            PopupMenuItem {
                height: Style.menuItemHeight
                labelText: Helper.noGroup
                width: parent.width
                leftPadding: Style.menuItemLeftPadding
                /// Disable the group in which the path already is so we can't move it in
                enabled: !(Helper.noGroup === myGroup)
                onTriggered: moveTo(Helper.noGroup)
            }

            Rectangle {
                color: Style.lightGreyBorder
                width: moveToMenu.width
                height: 1
            }

            ColumnLayout {
                anchors {
                    left: parent.left
                    right: parent.right
                }

                Repeater {
                    model: pathModel
                    delegate: PopupMenuItem {
                        anchors {
                            left: parent.left
                            right: parent.right
                        }
                        visible: groupName !== Helper.noGroup
                        Layout.preferredHeight: visible ? Style.menuItemHeight : 0
                        leftPadding: Style.menuItemLeftPadding
                        /// Disable the group in which the path already is so we can't move it in
                        enabled: !(groupName === myGroup)
                        labelText: {
                            console.log("number of robot = " + robotModel.count)
                            groupName
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
        height: 1
    }

    PopupMenuItem {
        labelText: "Delete Path"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: deletePath()
    }
}
