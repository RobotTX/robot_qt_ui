import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import QtQuick.Layouts 1.3

Rectangle {
    /// Name of the element (point or group)
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

    /// We look for the group in which this point is and if this group is displayed (isVisible)
    /// then we display its points
    visible: Helper.previousGroupIsVisible(this)

    /// if the group in which we are doesn't display its points, we hide it in the menu
    height: Helper.previousGroupIsVisible(this) ? 37 : 0

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
        anchors.leftMargin: (groupName != "No Group" && groupName != "") ? 45 : 20

        width: Style.smallBtnWidth
        height: Style.smallBtnHeight

        background: Rectangle {
            color: "transparent"
        }

        Image {
            asynchronous: true
            /// Change the image depending on whther or not it's a point or a group and if it's visible
            source: (groupName == "") ? (_isVisible ? "qrc:/icons/fold" : "qrc:/icons/unfold") : (_isVisible ? "qrc:/icons/visible" : "qrc:/icons/invisible")
            fillMode: Image.Pad // For not stretching image
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
        }

        //onClicked: leftButtonClicked(parent)
        onClicked: myList.hideShow(name, groupName, _isVisible);
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
        //onClicked: myList.deletePointOrGroup(name, groupName)
    }
/*
    function leftButtonClicked(item){
        item.isVisible = !item.isVisible;
        myList.hideShow(name, groupName, item.isVisible);
    }*/
}
