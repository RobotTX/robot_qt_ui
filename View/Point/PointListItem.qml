import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import QtQuick.Layouts 1.3

Rectangle {
    id: pointListItem

    property Points pointModel

    /// Name of the point or group
    property string name: _name

    /// Name of the parent of the element (if it's a point => the group name)
    property string groupName: _groupName

    /// For a point => if the point is displayed on the map
    /// For a group => if the group is opened and displaying his points in the menu
    property bool isVisible: _isVisible

    /// Whether or not this item is the selected one in the list
    property bool isCurrentItem: ListView.isCurrentItem

    /// The list we are in
    property ListView myList

    signal hideShow(string name, string groupName, bool isVisible)
    signal rightButtonClicked(string name, string groupName)
    signal deletePoint(string name, string groupName)
    signal deleteGroup(string name)
    signal renameGroup(string name)

    /// We look for the group in which this point is and if this group is displayed (isVisible)
    /// then we display its points
    visible: Helper.isVisible(this, _groupName)

    /// if the group in which we are doesn't display its points, we hide it in the menu
    height: Helper.isVisible(this, _groupName) ? 37 : 0

    /// The blue rectangle on the selected item
    Rectangle {
        anchors.verticalCenter: parent.verticalCenter
        height: parent.height - 10
        anchors.left: parent.left
        anchors.right: parent.right
        color: isCurrentItem ? "#b0c8f7" : "transparent"
    }

    /// Make the element transparent so we use the backround color of its parent
    color: "transparent"

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
        anchors.leftMargin: (_groupName != "No Group" && _groupName != "") ? 45 : 20

        width: Style.smallBtnWidth
        height: Style.smallBtnHeight

        background: Rectangle {
            color: "transparent"
        }

        Image {
            asynchronous: true
            /// Change the image depending on whether or not it's a point or a group and if it's visible
            source: (_groupName == "") ? (_isVisible ? "qrc:/icons/fold" : "qrc:/icons/unfold") : (_isVisible ? "qrc:/icons/visible" : "qrc:/icons/invisible")
            fillMode: Image.Pad // For not stretching image
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
        }

        onClicked: myList.hideShow(_name, _groupName, _isVisible);
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
            onDeleteGroup: pointListItem.deleteGroup(name)
            onRenameGroup: pointListItem.renameGroup(name)
        }

        EditPointPopupMenu {
            id: editPointPopupMenu
            x: rightButton.width
            onDeletePoint: pointListItem.deletePoint(name, groupName)
            pointModel: pointListItem.pointModel
            myGroup: groupName
        }
    }
}
