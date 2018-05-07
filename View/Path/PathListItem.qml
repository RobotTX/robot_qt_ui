import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Path"
import "../../Model/Robot"
import "../Custom"

Column {
    id: groupListItem

    property Paths pathModel
    property Column column
    property Robots robotModel
    property string langue
    signal renameGroup(string name)
    signal editPath(string name, string groupName)

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
            color: (column.selectedGroup === groupName && column.selectedPath === "") ? Style.selectedItemColor : "transparent"
        }

        MouseArea {
            onClicked: {
                column.selectedGroup = groupName;
                column.selectedPath = "";
            }
            anchors.fill: parent
        }

        /// The left button in each element of the list
        SmallButton {
            id: leftButton
            imgSrc: groupIsOpen ? "qrc:/icons/fold" : "qrc:/icons/unfold"
            anchors {
                top: parent.top
                left: parent.left
                bottom: parent.bottom
                leftMargin: 20
            }

            onClicked: pathModel.hideShowGroup(groupName);
        }

        /// The item displaying the name of the path/group
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
                column.selectedPath = "";
                editGroupPopupMenu.open();
            }

            EditPathGroupPopupMenu {
                id: editGroupPopupMenu
                x: rightButton.width
                langue: groupListItem.langue
                onDeleteGroup: pathModel.deleteGroup(groupName)
                onRenameGroup: groupListItem.renameGroup(groupName)
            }
        }
    }

    Repeater {
        model: paths
        delegate: delegatePaths
    }

    Component {
        id: delegatePaths
        Column {
            Frame {
                visible: groupIsOpen
                height: visible ? 37 : 0
                width : groupListItem.width
                padding: 0

                /// The blue rectangle on the selected item
                background: Rectangle {
                    anchors.verticalCenter: parent.verticalCenter
                    height: parent.height - 10
                    anchors.left: parent.left
                    anchors.right: parent.right
                    color: (column.selectedGroup === groupName && column.selectedPath === pathName) ? Style.selectedItemColor : "transparent"
                }

                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        column.selectedGroup = groupName;
                        column.selectedPath = pathName;
                    }
                }

                /// The left button in each element of the list
                SmallButton {
                    id: leftButton2
                    tooltip: { if (pathIsVisible) {
                               langue == "English" ? "在地图上隐藏路径" : "Hide the path on the map"
                                } else {
                               langue == "English" ? "在地图上显示路径" : "Show the path on the map"
                            }
                    }
                    imgSrc: pathIsVisible ? "qrc:/icons/visible" : "qrc:/icons/invisible"
                    anchors {
                        top: parent.top
                        left: parent.left
                        bottom: parent.bottom
                        leftMargin: groupName === Helper.noGroup ? 20 : 45
                    }

                    onClicked: pathModel.hideShowPathOnMap(groupName, pathName);
                }

                /// The item displaying the name of the path/group
                CustomLabel {
                    text: qsTr(pathName)
                    color: Style.blackMenuTextColor
                    anchors {
                        verticalCenter: parent.verticalCenter
                        left: leftButton2.right
                        right: rightOpenPath.left
                        leftMargin: 5
                        rightMargin: 5
                    }
                }

                SmallButton {
                    id: rightOpenPath
                    imgSrc: pathIsOpen ? "qrc:/icons/fold" : "qrc:/icons/unfold"
                    tooltip: { if (pathIsOpen) {
                               langue == "English" ? "在地图上隐藏路径" : "Hide the path detail"
                                } else {
                               langue == "English" ? "在地图上显示路径" : "Show the path detail"
                            }
                    }
                    anchors {
                        top: parent.top
                        bottom: parent.bottom
                        right: rightMenuButton.left
                        rightMargin: 5
                    }

                    onClicked: pathModel.hideShowPath(groupName, pathName);
                }

                SmallButton {
                    id: rightMenuButton
                    imgSrc: "qrc:/icons/more"
                    anchors {
                        top: parent.top
                        bottom: parent.bottom
                        right: parent.right
                        rightMargin: 20
                    }

                    onClicked: {
                        column.selectedGroup = groupName;
                        column.selectedPath = pathName;
                        editPathPopupMenu.open();
                    }

                    EditPathPopupMenu {
                        id: editPathPopupMenu
                        x: rightButton.width
                        pathModel: groupListItem.pathModel
                        robotModel: groupListItem.robotModel
                        langue: groupListItem.langue
                        myGroup: groupName
                        onDeletePath: pathModel.deletePath(groupName, pathName)
                        onMoveTo: pathModel.moveTo(pathName, groupName, newGroup)
                        onEditPath: groupListItem.editPath(pathName, groupName)
                    }
                }
            }

            Repeater {
                model: pathPoints
                delegate: delegatePathPoint
            }

            Component {
                id: delegatePathPoint
                Frame {
                    visible: pathIsOpen && groupIsOpen
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

                    Rectangle {
                        height: 15
                        width: 2
                        color: "#d7d7d7"
                        anchors.horizontalCenter: rect.horizontalCenter
                        anchors.bottom: rect.top
                        visible: pathIsOpen && groupIsOpen && index > 0
                    }

                    Rectangle {
                        id: rect
                        height: 10
                        width: height
                        radius: height
                        color: "#d7d7d7"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.left: parent.left
                        anchors.leftMargin: groupName === Helper.noGroup ? 25 : 50
                        visible: pathIsOpen && groupIsOpen
                    }

                    /// The item displaying the name of the path/group
                    CustomLabel {
                        text: qsTr(name)
                        font.pixelSize: 14
                        color: Style.midGrey2
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.left: rect.right
                        anchors.right: parent.right
                        anchors.leftMargin: 10
                        anchors.rightMargin: 5
                    }

                    CustomLabel {
                        id: customLabelWaitTime
                        text: waitTime === -1 ? qsTr(langue == "English" ? "人为干预" : "Human Action") : qsTr(langue == "English" ? "等待: " + waitTime + "秒" :  "Delay : "+ waitTime +"秒")
                        horizontalAlignment: Text.AlignRight
                        font.pixelSize: 14
                        color: Style.midGrey2
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.right: parent.right
                        anchors.rightMargin: 5
                    }

                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            column.selectedGroup = groupName;
                            column.selectedPath = pathName;
                        }
                    }
                }
            }
        }
    }
}
