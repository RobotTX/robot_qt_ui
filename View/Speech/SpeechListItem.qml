import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Speech"
import "../../Model/Robot"
import "../Custom"

Column {
    id: groupListItem

    property Speechs speechModel
    property Column column
    property Robots robotModel
    property string langue
    property int menuIndex: -1
    signal renameGroup(string name)
    signal editSpeech(string name, string groupName)

    Frame {
        id: groupItem
        visible: !(groupName === Helper.noGroup)
        height: visible ? 37 : 0
        padding: 0
        anchors.left: parent.left
        anchors.right: parent.right
        background: Rectangle {
            anchors.fill: parent
            color: "transparent"
        }

        /// The blue rectangle on the selected item
        Rectangle {
            anchors.verticalCenter: parent.verticalCenter
            height: parent.height - 10
            anchors.left: parent.left
            anchors.right: parent.right
            color: (column.selectedGroup === groupName && column.selectedSpeech === "") ? Style.selectedItemColor : "transparent"
        }

        MouseArea {
            onClicked: {
                column.selectedGroup = groupName;
                column.selectedSpeech = "";
            }
            anchors.fill: parent
        }

        /// The left button in each element of the list
        SmallButton {
            id: leftButton
            imgSrc: isOpen ? "qrc:/icons/fold" : "qrc:/icons/unfold"
            anchors {
                top: parent.top
                left: parent.left
                bottom: parent.bottom
                leftMargin: 20
            }

            onClicked: speechModel.hideShowGroup(groupName);
        }

        /// The item displaying the name of the speech/group
        CustomLabel {
            text: qsTr(groupName)
            color: Style.blackMenuTextColor
            anchors.verticalCenter: parent.verticalCenter
            anchors.left: leftButton.right
            anchors.right: rightButton.left
            anchors.leftMargin: 5
            anchors.rightMargin: 5
        }

        SmallButton {
            id: rightButton
            imgSrc: "qrc:/icons/more"
            anchors {
                top: parent.top
                bottom: parent.bottom
                right: parent.right
            }
            anchors.rightMargin: 20

            onClicked: {
                column.selectedGroup = groupName;
                column.selectedSpeech = "";
                editGroupPopupMenu.open();
            }

            EditSpeechGroupPopupMenu {
                id: editGroupPopupMenu
                x: rightButton.width
                langue: groupListItem.langue
                onDeleteGroup: speechModel.deleteGroup(groupName)
                onRenameGroup: groupListItem.renameGroup(groupName)
            }
        }
    }

    Repeater {
        model: speechs
        delegate: delegateSpeechs
    }

    Component {
        id: delegateSpeechs
        Column {
            Frame {
                visible: isOpen && name.indexOf("file:") !== -1 ? false : true
                height: visible ? 37 : 0
                width : groupListItem.width
                padding: 0

                /// The blue rectangle on the selected item
                background: Rectangle {
                    visible: name.indexOf("file:") !== -1 ? false : true
                    anchors.verticalCenter: parent.verticalCenter
                    height: parent.height - 10
                    anchors.left: parent.left
                    anchors.right: parent.right
                    color: (column.selectedGroup === groupName && column.selectedSpeech === name) ? Style.selectedItemColor : "transparent"
                }

                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        column.selectedGroup = groupName;
                        column.selectedSpeech = name;
                    }
                }

                /// The item displaying the name of the speech/group
                CustomLabel {
                    text: qsTr(name)
                    visible: name.indexOf("file:") !== -1 ? false : true
                    color: Style.blackMenuTextColor
                    anchors {
                        verticalCenter: parent.verticalCenter
                        left: parent.left
                        right: rightOpenSpeech.left
                        leftMargin: 45
                        rightMargin: 5
                    }
                }

                SmallButton {
                    id: rightOpenSpeech
                    visible: name.indexOf("file:") !== -1 ? false : true
                    imgSrc: descriptionIsOpen ? "qrc:/icons/fold" : "qrc:/icons/unfold"
                    tooltip: { if (descriptionIsOpen) {
                               langue == "English" ? "Hide Speech Detail" : "隐藏语音细节"
                                } else {
                               langue == "English" ? "Show Speech Detail" : "查看语音细节"
                            }
                    }
                    anchors {
                        top: parent.top
                        bottom: parent.bottom
                        right: rightMenuButton.left
                        rightMargin: 5
                    }

                    onClicked: {
                        column.selectedGroup = groupName
                        column.selectedSpeech = name
                        speechModel.hideShowDescription(groupName, name);
                    }
                }

                SmallButton {
                    id: rightMenuButton
                    visible: name.indexOf("file:") !== -1 ? false : true
                    imgSrc: "qrc:/icons/more"
                    anchors {
                        top: parent.top
                        bottom: parent.bottom
                        right: parent.right
                        rightMargin: 20
                    }

                    onClicked: {
                        column.selectedGroup = groupName;
                        column.selectedSpeech = name;
                        menuIndex = -1;
                        editSpeechPopupMenu1.open();
                    }

                    EditSpeechPopupMenu {
                        id: editSpeechPopupMenu1
                        x: rightButton.width
                        currentMenuIndex: groupListItem.menuIndex
                        speechModel: groupListItem.speechModel
                        robotModel: groupListItem.robotModel
                        langue: groupListItem.langue
                        myGroup: groupName
                        onDeleteSpeech: {
                            speechModel.deleteSpeech(myGroup, name);
                            speechModel.deleteSpeechSignal(myGroup, name);
                        }
                        onMoveTo: speechModel.moveTo(name, groupName, newGroup)
                        onEditSpeech: groupListItem.editSpeech(name, groupName)
                        onDoNothing: {
                            menuIndex = -1;
                            editSpeechPopupMenu2.open();
                        }
                    }

                    EditSpeechPopupMenu {
                        id: editSpeechPopupMenu2
                        x: rightButton.width
                        currentMenuIndex: groupListItem.menuIndex
                        speechModel: groupListItem.speechModel
                        robotModel: groupListItem.robotModel
                        langue: groupListItem.langue
                        myGroup: groupName
                        onDeleteSpeech: {
                            speechModel.deleteSpeech(myGroup, name);
                            speechModel.deleteSpeechSignal(myGroup, name);
                        }
                        onMoveTo: speechModel.moveTo(name, groupName, newGroup)
                        onEditSpeech: groupListItem.editSpeech(name, groupName)
                        onDoNothing: {
                            menuIndex = -1;
                            editSpeechPopupMenu1.open();
                        }
                    }
                }
            }

            Frame {
                visible: descriptionIsOpen
                height: visible ? 25 : 0
                anchors {
                    left: parent.left
                    right: parent.right
                    rightMargin: 35
                }
                padding: 0
                background: Rectangle {
                    anchors.fill: parent
                    color: "transparent"
                }

                CustomLabel {
                    text:langue == "English" ? "text: " +tts :"正文: " + tts
                    height: 15
                    font.pixelSize: 14
                    wrapMode: Label.WordWrap
                    width: parent.width
                    color: Style.midGrey2
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.left: parent.left
                    anchors.leftMargin: groupName === Helper.noGroup ? 25 : 50
                    visible: descriptionIsOpen && isOpen
                }

                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        column.selectedGroup = groupName;
                        column.selectedSpeech = speechName;
                    }
                }
            }
        }
    }
}
