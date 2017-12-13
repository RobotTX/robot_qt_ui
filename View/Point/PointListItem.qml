import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../../Model/Robot"
import "../Custom"

Column {
    id: groupListItem

    property Points pointModel
    property Column column
    property Robots robotModel
    property string langue

    signal renameGroup(string name)
    signal editPoint(string name, string groupName)

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
            color: (column.selectedGroup === groupName && column.selectedPoint === "") ? Style.selectedItemColor : "transparent"
        }

        MouseArea {
            onClicked: {
                column.selectedGroup = groupName;
                column.selectedPoint = "";
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

            onClicked: pointModel.hideShowGroup(groupName);
        }

        /// The item displaying the name of the point/group
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
                column.selectedPoint = ""
                editGroupPopupMenu.open()
            }

            EditPointGroupPopupMenu {
                id: editGroupPopupMenu
                x: rightButton.width
                langue: groupListItem.langue
                onDeleteGroup: pointModel.deleteGroup(groupName)
                onRenameGroup: groupListItem.renameGroup(groupName)
            }
        }
    }

    Repeater {
        id: pointList
        anchors {
            left: parent.left
            top: groupItem.bottom
            right: parent.right
        }

        model: points
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
                color: (column.selectedGroup === groupName && column.selectedPoint === name) ? Style.selectedItemColor : "transparent"
            }

            MouseArea {
                onClicked: {
                    column.selectedGroup = groupName;
                    column.selectedPoint = name;
                }
                anchors.fill: parent
            }

            /// The left button in each element of the list
            SmallButton {
                id: leftButton2
                tooltip: isVisible ? "Hide the point on the map" : "Show the point on the map"
                imgSrc: isVisible ? "qrc:/icons/visible" : "qrc:/icons/invisible"
                anchors {
                    top: parent.top
                    left: parent.left
                    bottom: parent.bottom
                    leftMargin: groupName === Helper.noGroup ? 20 : 45
                }

                onClicked: pointModel.hideShowPoint(groupName, name);
            }

            /// The item displaying the name of the point/group
            CustomLabel {
                text: qsTr(name)
                color: Style.blackMenuTextColor
                anchors.verticalCenter: parent.verticalCenter
                anchors.left: leftButton2.right
                anchors.right: rightMenuButton.left
                anchors.leftMargin: 5
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
                    column.selectedPoint = name
                    editPointPopupMenu.open();
                }

                EditPointPopupMenu {
                    id: editPointPopupMenu
                    x: rightButton.width
                    pointModel: groupListItem.pointModel
                    robotModel: groupListItem.robotModel
                    langue: groupListItem.langue
                    myGroup: groupName
                    onDeletePoint: {
                        pointModel.deletePoint(myGroup, name);
                        pointModel.deletePointSignal(myGroup, name);
                    }
                    onMoveTo: pointModel.moveTo(name, groupName, newGroup)
                    onEditPoint: groupListItem.editPoint(name, groupName)
                }
            }
        }
    }
}
