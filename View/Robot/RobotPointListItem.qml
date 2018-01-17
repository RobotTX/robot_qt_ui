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
    property bool open: false

    signal renameGroup(string name)
    signal editPoint(string name, string groupName)

    Rectangle {
        id: groupItem
        visible: !(name === Helper.noRobot)
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
//            color: (column.selectedGroup === name && column.selectedPoint === "") ? Style.selectedItemColor : "transparent"
        }

//        MouseArea {
//            onClicked: {
//                column.selectedGroup = groupName;
//                column.selectedPoint = "";
//            }
//            anchors.fill: parent
//        }

        /// The left button in each element of the list
        SmallButton {
            id: leftButton
            imgSrc: if (open) {
                        "qrc:/icons/fold" ;
                    } else {
                        "qrc:/icons/unfold";
                    }
            anchors {
                top: parent.top
                left: parent.left
                bottom: parent.bottom
                leftMargin: 20
            }

            onClicked: {
                if (open === false) {
                    open = true
                } else {
                    open = false
                }
            }
        }

        /// The item displaying the name of the point/group
        CustomLabel {
            text: qsTr(name)
            color: Style.blackMenuTextColor
            anchors.verticalCenter: parent.verticalCenter
            anchors.left: leftButton.right
            anchors.leftMargin: 5
        }
    }
}
