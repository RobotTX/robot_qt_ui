import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"

Column {
    id: groupListItem

    property Points pointModel
    property Column column
    signal rightButtonClicked(string name, string groupName)
    signal deletePoint(string name, string groupName)
    signal deleteGroup(string name)
    signal renameGroup(string name)
    signal moveTo(string name, string oldGroup, string newGroup)
    signal editPoint(string name, string groupName)
    signal hideShowGroup(string groupName)
    signal hideShowPoint(string groupName, string name)

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
        Button {
            id: leftButton
            anchors {
                top: parent.top
                left: parent.left
                bottom: parent.bottom
                leftMargin: 20
            }

            width: Style.smallBtnWidth
            height: Style.smallBtnHeight

            background: Rectangle {
                color: "transparent"
            }

            Image {
                asynchronous: true
                /// Change the image depending on whether or not it's a point or a group and if it's visible
                source: isOpen ? "qrc:/icons/fold" : "qrc:/icons/unfold"
                fillMode: Image.Pad // For not stretching image
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }

            onClicked: hideShowGroup(groupName);
        }

        /// The item displaying the name of the point/group
        Label {
            text: qsTr(groupName)
            color: Style.blackMenuTextColor
            anchors.verticalCenter: parent.verticalCenter
            anchors.left: leftButton.right
            anchors.right: rightButton.left
            anchors.leftMargin: 5
            anchors.rightMargin: 5
            maximumLineCount: 1
            elide: Text.ElideRight
        }

        Button {
            id: rightButton
            anchors {
                top: parent.top
                bottom: parent.bottom
                right: parent.right
            }
            anchors.rightMargin: 20

            width: Style.smallBtnWidth
            height: Style.smallBtnHeight

            background: Rectangle {
                color: "transparent"
            }

            Image {
                asynchronous: true
                source: "qrc:/icons/more"
                fillMode: Image.Pad // For not stretching image
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }

            onClicked: editGroupPopupMenu.open()

            EditGroupPopupMenu {
                id: editGroupPopupMenu
                x: rightButton.width
                onDeleteGroup: groupListItem.deleteGroup(groupName)
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

        objectName: "pointList"
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
            Button {
                id: leftButton2
                anchors {
                    top: parent.top
                    left: parent.left
                    bottom: parent.bottom
                    leftMargin: groupName === Helper.noGroup ? 20 : 45
                }

                width: Style.smallBtnWidth
                height: Style.smallBtnHeight

                background: Rectangle {
                    color: "transparent"
                }

                Image {
                    asynchronous: true
                    /// Change the image depending on whether or not it's a point or a group and if it's visible
                    source: isVisible ? "qrc:/icons/visible" : "qrc:/icons/invisible"
                    fillMode: Image.Pad // For not stretching image
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                }

                onClicked: hideShowPoint(groupName, name);
            }

            /// The item displaying the name of the point/group
            Label {
                text: qsTr(name)
                color: Style.blackMenuTextColor
                anchors.verticalCenter: parent.verticalCenter
                anchors.left: leftButton2.right
                anchors.right: rightButton2.left
                anchors.leftMargin: 5
                anchors.rightMargin: 5
                maximumLineCount: 1
                elide: Text.ElideRight
            }

            Button {
                id: rightButton2
                anchors {
                    top: parent.top
                    bottom: parent.bottom
                    right: parent.right
                }
                anchors.rightMargin: 20

                width: Style.smallBtnWidth
                height: Style.smallBtnHeight

                background: Rectangle {
                    color: "transparent"
                }

                Image {
                    asynchronous: true
                    source: "qrc:/icons/more"
                    fillMode: Image.Pad // For not stretching image
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                }

                onClicked: editPointPopupMenu.open();

                EditPointPopupMenu {
                    id: editPointPopupMenu
                    x: rightButton.width
                    pointModel: groupListItem.pointModel
                    myGroup: groupName
                    onDeletePoint: groupListItem.deletePoint(name, groupName)
                    onMoveTo: groupListItem.moveTo(name, groupName, newGroup)
                    onEditPoint: groupListItem.editPoint(name, groupName)
                }
            }
        }
    }
}
