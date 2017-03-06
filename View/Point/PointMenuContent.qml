import QtQuick 2.7
import QtQuick.Controls 2.0
import "../../Helper/style.js" as Style
import "../Custom"
import "../../Model/Point"

Frame {
    id: pointMenuFrame
    padding: 0

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    /// This frame is displayed when there is no point
    EmptyMenu {
        visible: pointList.count == 0
        txt: "You have no points, click the '+' button or double click the map to create a point."
        imgSrc: "qrc:/icons/big_point"
    }

    /// This frame is displayed when we have points
    Component {
        id: delegate
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

            /// We look for the group in which this point is and if this group is displayed (isVisible)
            /// then we display its points
            visible: previousGroupIsVisible(this)

            /// if the group in which we are doesn't display its points, we hide it in the menu
            height: previousGroupIsVisible(this) ? 37 : 0

            width: pointMenuFrame.width


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
                onClicked: pointList.currentIndex = index;
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
                    /// Change the image depending on whther or not it's a point or a group and if it's visible
                    source: (groupName == "") ? (isVisible ? "qrc:/icons/fold" : "qrc:/icons/unfold") : (isVisible ? "qrc:/icons/visible" : "qrc:/icons/invisible")
                    fillMode: Image.Pad // For not stretching image
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                }

                onClicked: leftButtonClicked(parent)
            }

            /// The item displaying the name of the point/group
            Label {
                text: name
                color: Style.blackMenuTextColor
                anchors.verticalCenter: parent.verticalCenter
                anchors.left: leftButton.right
                anchors.leftMargin: 5
            }

            /// The right button
            Button {
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
                    source: "qrc:/icons/more"
                    fillMode: Image.Pad // For not stretching image
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                }
                onClicked: console.log("Clicked on the right button in the point menu : " + name + " in the group " + groupName)
            }
        }
    }

    /// The list containing both the graphical and model of the points in the menu
    ListView {
        id: pointList
        anchors.fill: parent
        model: Point {}
        delegate: delegate
        focus: true
        anchors.topMargin: 14
    }

    /// To get the current index of this item
    function itemIndex(item) {
        /// if item is not parented, -1 is returned
        if (item.parent === null)
            return -1
        var siblings = item.parent.children
        for (var i = 0; i < siblings.length; i++)
            if (siblings[i] === item)
                return i
        return -1 //will item.never happen
    }

    /// To check whether or not the group we are in is displayed or not
    function previousGroupIsVisible(item) {
        /// We always display in the menu the points in "No Group"
        /// + we always display the groups
        if(item.groupName === "No Group" || item.groupName === "")
            return true;

        if (item.parent === null)
            return false;

        var index = itemIndex(item)
        if(index > 0){
            /// If the previous item is a group,
            /// it is OUR group so we check if we display the point
            if (item.parent.children[itemIndex(item) - 1].groupName === ""){
                if(item.parent.children[itemIndex(item) - 1].isVisible)
                    return true;
                else
                    return false;
            } else
                /// If the previous item is a point,
                /// we look for its previous item until we find our group
                return previousGroupIsVisible(item.parent.children[itemIndex(item) - 1]);
        }

        return false;
    }

    function leftButtonClicked(item){
        item.isVisible = !item.isVisible;
    }
}
