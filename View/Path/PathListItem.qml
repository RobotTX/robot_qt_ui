import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Path"

Column {
    id: groupListItem

    property Paths pathModel
    property Column column
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
                /// Change the image depending on whether or not it's a path or a group and if it's visible
                source: groupIsOpen ? "qrc:/icons/fold" : "qrc:/icons/unfold"
                fillMode: Image.Pad // For not stretching image
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }

            onClicked: pathModel.hideShowGroup(groupName);
        }

        /// The item displaying the name of the path/group
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

            EditPathGroupPopupMenu {
                id: editGroupPopupMenu
                x: rightButton.width
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
                    onClicked: {
                        column.selectedGroup = groupName;
                        column.selectedPath = pathName;
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
                        /// Change the image depending on whether or not it's a path or a group and if it's visible
                        source: pathIsVisible ? "qrc:/icons/visible" : "qrc:/icons/invisible"
                        fillMode: Image.Pad // For not stretching image
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.verticalCenter: parent.verticalCenter
                    }

                    onClicked: pathModel.hideShowPathOnMap(groupName, pathName);
                }

                /// The item displaying the name of the path/group
                Label {
                    text: qsTr(pathName)
                    color: Style.blackMenuTextColor
                    anchors {
                        verticalCenter: parent.verticalCenter
                        left: leftButton2.right
                        right: rightOpenPath.left
                        leftMargin: 5
                        rightMargin: 5
                    }

                    maximumLineCount: 1
                    elide: Text.ElideRight
                }

                Button {
                    id: rightOpenPath
                    anchors {
                        top: parent.top
                        bottom: parent.bottom
                        right: rightMenuButton.left
                        rightMargin: 5
                    }

                    width: Style.smallBtnWidth
                    height: Style.smallBtnHeight

                    background: Rectangle {
                        color: "transparent"
                    }

                    Image {
                        asynchronous: true
                        /// Change the image depending on whether or not it's a path or a group and if it's visible
                        source: pathIsOpen ? "qrc:/icons/fold" : "qrc:/icons/unfold"
                        fillMode: Image.Pad // For not stretching image
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.verticalCenter: parent.verticalCenter
                    }

                    onClicked: pathModel.hideShowPath(groupName, pathName);
                }

                Button {
                    id: rightMenuButton
                    anchors {
                        top: parent.top
                        bottom: parent.bottom
                        right: parent.right
                        rightMargin: 20
                    }

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

                    onClicked: editPathPopupMenu.open();

                    EditPathPopupMenu {
                        id: editPathPopupMenu
                        x: rightButton.width
                        pathModel: groupListItem.pathModel
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
                    visible: pathIsOpen
                    height: visible ? 25 : 0
                    anchors.left: parent.left
                    anchors.right: parent.right
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
                        visible: index > 0
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
                    }

                    /// The item displaying the name of the path/group
                    Label {
                        text: qsTr(name)
                        font.pixelSize: 14
                        color: Style.midGrey2
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.left: rect.right
                        anchors.right: parent.right
                        anchors.leftMargin: 10
                        anchors.rightMargin: 5
                        maximumLineCount: 1
                        elide: Text.ElideRight
                    }
                }
            }
        }
    }
}
