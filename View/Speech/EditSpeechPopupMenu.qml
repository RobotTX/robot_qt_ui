import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Speech"
import "../../Model/Robot"
import "../Custom"
import "../Robot"

Menu {
    id: menu
    padding: 0
    width: 188
    property bool _isVisible

    property Speechs speechModel
    property Robots robotModel
    property string myGroup
    property int currentMenuIndex: -1
    property string langue

    signal editSpeech()
    signal deleteSpeech(string name)
    signal moveTo(string newGroup)

    visible: _isVisible

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    PopupMenuItem {
        id: sendTtsToRobot
        labelText: langue == "English" ? "Send speech to robot" : "Send speech to robot"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
//        visible: myGroup == "Gobot"
//        enabled: myGroup == "Gobot"
        visible: true
        enabled: true
        Image {
            asynchronous: true
            source: "qrc:/icons/arrow"
            fillMode: Image.Pad // For not stretching image
            anchors.verticalCenter: parent.verticalCenter
            anchors.right: parent.right
            anchors.rightMargin: 12
//            visible: myGroup == "Gobot"
            visible: true
        }
        onHoveredChanged: if (visible) {
                              currentMenuIndex = 0
                          } /// desktop
//        onClicked: if (visible) {
//                       currentMenuIndex = 0
//                   } /// android

        property string nameSpeech: name
        property string ttsSpeech: tts

        RobotListInPopup {
            id: robotListInPopup
            x: sendTtsToRobot.width
            visible: menu.currentMenuIndex === 0
            robotModel: menu.robotModel
            onRobotSelected: {
                robotModel.sendTtsToRobot(ip, sendTtsToRobot.ttsSpeech);
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
        labelText: langue == "English" ? "Edit Speech" : "Edit Speech"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: {
            console.log("button edit pressed");
            editSpeech()
        }
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    PopupMenuItem {
        labelText: langue == "English" ? "移动到" : "Move to Group"
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
        onHoveredChanged: if (visible) { currentMenuIndex = 1} /// desktop
//        onClicked: if (visible) { currentMenuIndex = 1} /// android


        Menu {
            id: moveToMenu
            padding: 0
            width: 140
            x: parent.width
            visible: menu.currentMenuIndex === 1

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
                /// Disable the group in which the speech already is so we can't move it in
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
                    model: speechModel
                    delegate: PopupMenuItem {
                        anchors {
                            left: parent.left
                            right: parent.right
                        }
                        visible: groupName !== Helper.noGroup
                        Layout.preferredHeight: visible ? Style.menuItemHeight : 0
                        leftPadding: Style.menuItemLeftPadding
                        /// Disable the group in which the speech already is so we can't move it in
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
        labelText: langue == "English" ? "Delete Speech" : "Delete Speech"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: deleteSpeech(name)
    }
}
