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
    property int menuIndex: -1
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
            font.pointSize: Style.ubuntuHeading2Size
           // font.bold: true
            color: Style.groupNameBold
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
                               langue == "English" ? "Hide path on map" : "隐藏地图上该路径"
                                } else {
                               langue == "English" ? "Show path on map" : "显示地图上该路径"
                            }
                    }
                    imgSrc: pathIsVisible ? "qrc:/icons/visible" : "qrc:/icons/invisible"
                    anchors {
                        top: parent.top
                        left: parent.left
                        bottom: parent.bottom
                        leftMargin: groupName === Helper.noGroup ? 20 : 45
                    }

                    onClicked: {
                        console.log("hide show");
                        pathModel.hideShowPathOnMap(groupName, pathName);
                    }
                }

                /// The item displaying the name of the path/group
                CustomLabel {
                    text: qsTr(pathName)
                    font.pointSize: Style.ubuntuHeading2Size
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
                               langue == "English" ? "Hide path detail" : "隐藏路径细节"
                                } else {
                               langue == "English" ? "Show path detail" : "查看路径细节"
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
                        groupListItem.menuIndex = -1;
                        editPathPopupMenu1.open();
                    }

                    EditPathPopupMenu {
                        id: editPathPopupMenu1
                        x: rightButton.width
                        currentMenuIndex: groupListItem.menuIndex
                        pathModel: groupListItem.pathModel
                        robotModel: groupListItem.robotModel
                        langue: groupListItem.langue
                        myGroup: groupName
                        onDeletePath: pathModel.deletePath(groupName, pathName)
                        onMoveTo: pathModel.moveTo(pathName, groupName, newGroup)
                        onEditPath: groupListItem.editPath(pathName, groupName)
                        onDoNothing: {
                            menuIndex = -1;
                            editPathPopupMenu2.open();
                        }
                    }

                    /// we duplicate EditPathPopupMenu in order to do nothing
                    /// when the user click on one item
                    EditPathPopupMenu {
                        id: editPathPopupMenu2
                        x: rightButton.width
                        currentMenuIndex: groupListItem.menuIndex
                        pathModel: groupListItem.pathModel
                        robotModel: groupListItem.robotModel
                        langue: groupListItem.langue
                        myGroup: groupName
                        onDeletePath: pathModel.deletePath(groupName, pathName)
                        onMoveTo: pathModel.moveTo(pathName, groupName, newGroup)
                        onEditPath: groupListItem.editPath(pathName, groupName)
                        onDoNothing: {
                            menuIndex = -1;
                            editPathPopupMenu1.open();
                        }
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
                        text: {
                            console.log("name = " + name)
                            qsTr(name)
                        }
                        font.pointSize: Style.ubuntuSubHeadingSize
                        color: Style.midGrey2
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.left: rect.right
                        anchors.right: parent.right
                        anchors.leftMargin: 10
                        anchors.rightMargin: 5
                    }

                    CustomLabel {
                        id: customLabelWaitTime
                        text: waitTime === -1 ? qsTr(langue == "English" ? "Human Action" : "人为干预") : qsTr(langue == "English" ? "Delay: " + waitTime + "s" :  "等待 : "+ waitTime + "秒")
                        horizontalAlignment: Text.AlignRight
                        font.pointSize: Style.ubuntuSubHeadingSize
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
