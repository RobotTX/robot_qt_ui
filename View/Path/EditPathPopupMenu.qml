import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Path"
import "../../Model/Robot"
import "../Custom"
import "../Robot"

Menu {
    id: menu
    padding: 0
    width: 188

    property Paths pathModel
    property Robots robotModel
    property string myGroup
    property string myRobot
    property string langue
    property int currentMenuIndex

    signal newPathSignal(string ip, string groupName, string pathName)
    signal editPath()
    signal deletePath()
    signal moveTo(string newGroup)
    signal robotSelected(string robot)
    signal doNothing()

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    PopupMenuItem {
        id: assignPathBis
        height: Style.menuItemHeight
        width: parent.width
        labelText: langue == "English" ? "Send Path To" :"发送路径给"
        leftPadding: Style.menuItemLeftPadding
        visible: true
        enabled: true

        Image {
            asynchronous: true
            source: "qrc:/icons/arrow"
            fillMode: Image.Pad // For not stretching image
            anchors.verticalCenter: parent.verticalCenter
            anchors.right: parent.right
            anchors.rightMargin: 12
        }
        onHoveredChanged: if(visible){
                            robotListInPopup.open();
                            menu.currentMenuIndex = 0;
                          } /// destkop
        onTriggered: doNothing()

//        onClicked: if(visible){ currentMenuIndex = 0 } /// android

        RobotListInPopup {
            id: robotListInPopup
            x: assignPathBis.width
            visible: menu.currentMenuIndex === 0
            robotModel: menu.robotModel
            onRobotSelected: {
                robotModel.newPathSignal(ip, groupName, pathName);
                robotModel.pathNameAssigned = pathName;
                robotModel.nameRobotPath = name;
                robotModel.robotIP = ip;
                robotModel.robotSelected = true; /// for console in robotView
//                currentMenuIndex = -1;
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
        labelText: langue == "English" ? "Edit Path" : "修改路径"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onHoveredChanged: if (visible) menu.currentMenuIndex = 1
        onClicked: if(visible){ menu.currentMenuIndex = 1 }
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
        labelText: langue == "English" ? "Move To Group" : "移动到"
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

        onHoveredChanged: if (visible) {
                              menuGroup.open();
                              menu.currentMenuIndex = 2
                          }/// desktop
        onTriggered: doNothing()
//        onClicked: if(visible){ currentMenuIndex = 2 } /// android

        Menu {
            id: menuGroup
            x: parent.width
            visible: menu.currentMenuIndex === 2
            padding: 0
            width: 140

            background: Rectangle {
                color: Style.lightGreyBackground
                border.color: Style.lightGreyBorder
                radius: 5
            }

            PopupMenuItem {
                height: Style.menuItemHeight
                labelText: langue == "English" ? Helper.noGroup : Helper.noGroupChinese
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
        labelText: langue == "English" ? "Delete Path" : "删除路径"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onHoveredChanged: if (visible) { currentMenuIndex = 3}
//        onClicked: if(visible){ currentMenuIndex = 3 }
        onTriggered: deletePath()
    }
}
