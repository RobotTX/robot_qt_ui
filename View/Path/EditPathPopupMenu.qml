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

    signal newPathSignal(string ip, string groupName, string pathName)
    signal editPath()
    signal deletePath()
    signal moveTo(string newGroup)
    signal robotSelected(string robot)

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    PopupMenuItem {
        id: assignPathBis
        height: Style.menuItemHeight
        width: parent.width
        labelText: "Assign this path to"
        leftPadding: Style.menuItemLeftPadding

        Image {
            asynchronous: true
            source: "qrc:/icons/arrow"
            fillMode: Image.Pad // For not stretching image
            anchors.verticalCenter: parent.verticalCenter
            anchors.right: parent.right
            anchors.rightMargin: 12
        }
        onHoveredChanged: if(visible){ currentMenuIndex = 0 }

        RobotListInPopup {
            x: assignPathBis.width
            visible: menu.currentMenuIndex === 0
            onVisibleChanged: if(!visible) currentMenuIndex = -1
            robotModel: menu.robotModel
            onRobotSelected: {
                robotModel.newPathSignal(ip, groupName, pathName)
                currentMenuIndex = -1;
                menu.currentMenuIndex = -1;
                menu.close();
            }
        }
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    // first item
    PopupMenuItem {
        labelText: "Edit Path"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onHoveredChanged: if (visible) currentMenuIndex = 1
        onTriggered: editPath()
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    // third item
    PopupMenuItem {
        id: moveToMenu
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

        onHoveredChanged: if (visible) currentMenuIndex = 2

        Menu {
            x: parent.width
            visible: menu.currentMenuIndex === 2
            onVisibleChanged: menu.currentMenuIndex = -1
            padding: 0
            width: 140

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
        onHoveredChanged: if (visible) {

                              currentMenuIndex = 3
                          console.log("DELETE PATH SHOULD BE 3 : currentMenuIndex = " + currentMenuIndex + " visible = " + visible)}
        onTriggered: deletePath()
    }
}
