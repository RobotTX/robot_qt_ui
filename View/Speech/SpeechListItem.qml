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
    property Robots robotModel
    property Column column
    property string langue

    signal renameGroup(string name)
    signal editSpeech(string name, string groupName)

    Rectangle {
        id: groupItem
        visible: !(groupName === Helper.noGroup)
        height: visible ? 37 :0
        anchors.left: parent.left
        anchors.right: parent.right
        color: "transparent"

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
                column.selectedGroup = groupName
                column.selectedSpeech = ""
                editGroupPopupMenu.open()
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
        id: speechList
        anchors {
            left: parent.left
            top: groupItem.bottom
            right: parent.right
        }

        model: speechs
        delegate: delegate
        focus: true
        anchors.topMargin: 14
    }


    Component {

        id: delegate

        Rectangle {
            visible: isOpen
            height: isOpen ? 37 : 0
            anchors.left: parent.left
            anchors.right: parent.right
            color: "transparent"

            /// The blue rectangle on the selected item
            Rectangle {
                anchors.verticalCenter: parent.verticalCenter
                height: parent.height - 10
                anchors.left: parent.left
                anchors.right: parent.right
                color: (column.selectedGroup === groupName && column.selectedSpeech === name) ? Style.selectedItemColor : "transparent"
            }

            MouseArea {
                onClicked: {
                    column.selectedGroup = groupName;
                    column.selectedSpeech = name;
                }
                anchors.fill: parent
            }

            /// The item displaying the name of the speech/group
            CustomLabel {
                text: {
                    qsTr(name)
                }
                color: Style.blackMenuTextColor
                anchors.verticalCenter: parent.verticalCenter
                anchors.left: parent.left
//                anchors.right: rightMenuButton.left
                anchors.leftMargin: 50
                anchors.rightMargin: 5
            }

            SmallButton {
                id: rightMenuButton
                imgSrc: "qrc:/icons/more"
                anchors {
                    top: parent.top
                    bottom: parent.bottom
                    right: parent.right
                }
                anchors.rightMargin: 20

                onClicked: {
                    column.selectedGroup = groupName
                    column.selectedSpeech = name
                    editSpeechPopupMenu.open();
                }

                EditSpeechPopupMenu {
                    id: editSpeechPopupMenu
                    x: rightButton.width
                    speechModel: groupListItem.speechModel
                    robotModel: groupListItem.robotModel
                    langue: groupListItem.langue
                    myGroup: groupName
                    onDeleteSpeech: {
                        speechModel.deleteSpeech(myGroup, name);
                        speechModel.deleteSpeechSignal(myGroup, name);
                    }
                    onMoveTo: speechModel.moveTo(name, groupName, newGroup)
                    onEditSpeech: {
                        console.log("we are in speechlistitem editspeech");
                        groupListItem.editSpeech(name, groupName)
                    }
                }
            }
        }
    }
}
