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

    EmptyMenu {
        visible: pointList.count == 0
        txt: "You have no points, click the '+' button or double click the map to create a point."
        imgSrc: "qrc:/icons/big_point"
    }

    Component {
        id: delegate
        Rectangle {
            property string groupName: group
            property bool _isVisible: isVisible
            property bool isCurrentItem: ListView.isCurrentItem

            visible: previousGroupIsVisible(this)
            height: previousGroupIsVisible(this) ? 37 : 0
            width: pointMenuFrame.width


            Rectangle {
                anchors.verticalCenter: parent.verticalCenter
                height: parent.height - 10
                anchors.left: parent.left
                anchors.right: parent.right
                color: isCurrentItem ? "#b0c8f7" : "transparent"
            }
            color: "transparent"

            MouseArea {
                onClicked: pointList.currentIndex = index;
                anchors.fill: parent
            }

            Button {
                id: leftButton
                anchors {
                    top: parent.top
                    left: parent.left
                    bottom: parent.bottom
                }
                /// If it's a point in a group we add some more margin
                anchors.leftMargin: (group != "No Group" && group != "") ? 45 : 20

                width: Style.smallBtnWidth
                height: Style.smallBtnHeight

                background: Rectangle {
                    color: "transparent"
                }

                Image {
                    source: (group == "") ? (_isVisible ? "qrc:/icons/fold" : "qrc:/icons/unfold") : (_isVisible ? "qrc:/icons/visible" : "qrc:/icons/invisible")
                    fillMode: Image.Pad // For not stretching image
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                }

                onClicked: leftButtonClicked(parent)
            }

            Text {
                text: name
                color: "#262626"
                anchors.verticalCenter: parent.verticalCenter
                anchors.left: leftButton.right
                anchors.leftMargin: 5
            }

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
                onClicked: console.log("Clicked on the right button in the point menu : " + name + " in the group " + group)
            }
        }
    }

    ListView {
        id: pointList
        anchors.fill: parent
        model: Point {}
        delegate: delegate
        focus: true
        anchors.topMargin: 14
    }

    /// if item is not parented, -1 is returned
    function itemIndex(item) {
        if (item.parent == null)
            return -1
        var siblings = item.parent.children
        for (var i = 0; i < siblings.length; i++)
            if (siblings[i] == item)
                return i
        return -1 //will item.never happen
    }

    function previousGroupIsVisible(item) {
        if(item.groupName == "No Group" || item.groupName == "")
            return true;

        if (item.parent == null)
            return false;

        var index = itemIndex(item)
        if(index > 0){
            if (item.parent.children[itemIndex(item) - 1].groupName == ""){
                if(item.parent.children[itemIndex(item) - 1]._isVisible)
                    return true;
                else
                    return false;
            } else
                return previousGroupIsVisible(item.parent.children[itemIndex(item) - 1]);
        }

        return false;
    }

    function leftButtonClicked(item){
        console.log("Clicked on the left button in the point menu : " + item.groupName + " " + item._isVisible);
        item._isVisible = !item._isVisible;
    }
}
