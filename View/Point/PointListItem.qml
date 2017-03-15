import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"

Column {
    id: groupListItem

    signal hideShow(string name, string groupName, bool isVisible)
    signal rightButtonClicked(string name, string groupName)
    signal deletePoint(string name, string groupName)
    signal deleteGroup(string name)
    signal renameGroup(string name)
    signal moveTo(string name, string oldGroup, string newGroup)
    signal editPoint(string name, string groupName)

    Rectangle {
        height: 37
        anchors.left: parent.left
        anchors.right: parent.right
        color: "transparent"

        /// The left button in each element of the list
        Button {
            id: leftButton
            anchors {
                top: parent.top
                left: parent.left
                bottom: parent.bottom
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
        }

        /// The item displaying the name of the point/group
        Label {
            text: qsTr(name)
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
        }
    }

    Repeater {
        id: pointList
        objectName: "pointList"
        anchors.fill: parent
        model: points
        delegate: delegate
        focus: true
        anchors.topMargin: 14
    }


    Component {
        id: delegate
        Rectangle {
            visible: isOpen
            height: visible ? 37 : 0
            anchors.left: parent.left
            anchors.right: parent.right
            color: "transparent"

            /// The left button in each element of the list
            Button {
                id: leftButton2
                anchors {
                    top: parent.top
                    left: parent.left
                    bottom: parent.bottom
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
            }
        }
    }







    /*
    property Points pointModel

    /// Whether or not this item is the selected one in the list
    property bool isCurrentItem: ListView.isCurrentItem

    /// The list we are in
    property ListView myList
    /// We look for the group in which this point is and if this group is displayed (isVisible)
    /// then we display its points
    //visible: (_name === Helper.noGroup && _groupName === "") ? false : ((_groupName === "") ? true : _groupIsOpen)

    /// if the group in which we are doesn't display its points, we hide it in the menu
    height: visible ? 37 : 0


    /// The blue rectangle on the selected item
    Rectangle {
        anchors.verticalCenter: parent.verticalCenter
        height: parent.height - 10
        anchors.left: parent.left
        anchors.right: parent.right
        color: isCurrentItem ? Style.selectedItemColor : "transparent"
    }
    /// To be able to click on the item in the list to select it
    MouseArea {
        onClicked: myList.currentIndex = index;
        anchors.fill: parent
    }

    /// The left button in each element of the list
    Button {
        id: leftButton
        anchors {
            top: parent.top
            left: parent.left
            bottom: parent.bottom
        }
        /// If it's a point in a group we add some more margin
        anchors.leftMargin: (_groupName != Helper.noGroup && _groupName != "") ? 45 : 20

        width: Style.smallBtnWidth
        height: Style.smallBtnHeight

        background: Rectangle {
            color: "transparent"
        }

        Image {
            asynchronous: true
            /// Change the image depending on whether or not it's a point or a group and if it's visible
            source: (_groupName == "") ? (_groupIsOpen ? "qrc:/icons/fold" : "qrc:/icons/unfold") : (_isVisible ? "qrc:/icons/visible" : "qrc:/icons/invisible")
            fillMode: Image.Pad // For not stretching image
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
        }

        onClicked: myList.hideShow(_name, _groupName, (_groupName === "") ? _groupIsOpen : _isVisible);
    }

    /// The item displaying the name of the point/group
    Label {
        text: qsTr(_name)
        color: Style.blackMenuTextColor
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: leftButton.right
        anchors.right: rightButton.left
        anchors.leftMargin: 5
        anchors.rightMargin: 5
        maximumLineCount: 1
        elide: Text.ElideRight
    }

    /// The right button
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

        onClicked: {
            myList.currentIndex = index;
            _groupName === "" ? editGroupPopupMenu.open() : editPointPopupMenu.open();
        }

        EditGroupPopupMenu {
            id: editGroupPopupMenu
            x: rightButton.width
            onDeleteGroup: pointListItem.deleteGroup(_name)
            onRenameGroup: pointListItem.renameGroup(_name)
        }

        EditPointPopupMenu {
            id: editPointPopupMenu
            x: rightButton.width
            pointModel: pointListItem.pointModel
            myGroup: _groupName
            onDeletePoint: pointListItem.deletePoint(_name, _groupName)
            onMoveTo: pointListItem.moveTo(_name, _groupName, newGroup)
            onEditPoint: pointListItem.editPoint(_name, _groupName)
        }
    }*/
}
