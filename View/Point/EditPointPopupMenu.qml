import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../../Model/Robot"
import "../Custom"
import "../Robot"

Menu {
    id: menu
    padding: 0
    width: 188

    property Points pointModel
    property Robots robotModel
    property string myGroup
    property int currentMenuIndex: -1
    property string langue

    signal savePlaceSignal(string ip, string name, double x, double y, double orientation, bool home)
    signal editPoint()
    signal deletePoint(string name)
    signal moveTo(string newGroup)

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    PopupMenuItem {
        id: assignPointTo
        labelText: langue == "English" ? "Send point to robot" : "Send point to robot"
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
//        onHoveredChanged: if (visible) { currentMenuIndex = 0 }
        onHoveredChanged: if (visible) {
                              console.log("onhovered")
                              currentMenuIndex = 0
                          }

        RobotListInPopup {
            id: robotListInPopup
            x: assignPointTo.width
            visible: menu.currentMenuIndex === 0
            onVisibleChanged: if (!visible) currentMenuIndex = -1
            robotModel: menu.robotModel
            onRobotSelected: {
                robotModel.savePlaceSignal(ip, pointModel.namePoint, posX, posY, orientation, home)
                console.log("name = " + pointModel.namePoint + " posX = " + posX + " posY = " + posY + " home = " + home + " orientation = " + orientation)
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

    PopupMenuItem {
        labelText: langue == "English" ? "修改目标点" : "Edit Point"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: editPoint()
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    PopupMenuItem {
        labelText: langue == "English" ? "移动到" : "Move to"
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
//        onHoveredChanged: if(visible && !moveToMenu.visible) moveToMenu.open()
        onHoveredChanged: if (visible) { currentMenuIndex = 1}


        Menu {
            id: moveToMenu
            padding: 0
            width: 140
            x: parent.width
            visible: menu.currentMenuIndex === 1
            onVisibleChanged: menu.currentMenuIndex = -1

            background: Rectangle {
                color: Style.lightGreyBackground
                border.color: Style.lightGreyBorder
                radius: 5
            }

            PopupMenuItem {
                height: Style.menuItemHeight
                labelText: langue == "English" ? Helper.noGroupChinese : Helper.noGroup
                width: parent.width
                leftPadding: Style.menuItemLeftPadding
                /// Disable the group in which the point already is so we can't move it in
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
                    model: pointModel
                    delegate: PopupMenuItem {
                        anchors {
                            left: parent.left
                            right: parent.right
                        }
                        visible: groupName !== Helper.noGroup
                        Layout.preferredHeight: visible ? Style.menuItemHeight : 0
                        leftPadding: Style.menuItemLeftPadding
                        /// Disable the group in which the point already is so we can't move it in
                        enabled: !(groupName === myGroup)
                        labelText: groupName

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
        labelText: langue == "English" ? "删除目标点" : "Delete Point"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: deletePoint(name)
    }
}
