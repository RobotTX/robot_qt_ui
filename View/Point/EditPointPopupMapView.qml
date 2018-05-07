import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../../Model/Robot"
import "../Custom"
import "../Robot"

Menu {
    id: menu
    padding: 0
    width: 188

    property Points pointModel
    property Robots robotModel
    property string myGroup
    property int currentMenuIndex: -1
    property string langue

    signal savePlaceSignal(string ip, string name, double x, double y, double orientation, bool home)
    signal editPoint()
    signal deletePoint(string name)
    signal moveTo(string newGroup)

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    PopupMenuItem {
        id: assignPointTo
        labelText: langue == "English" ? "发目标点给机器人" : "Send Point To"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
//        visible: myGroup == "Gobot"
//        enabled: myGroup == "Gobot"
        visible: true
        enabled: true
        Image {
            asynchronous: true
            source: "qrc:/icons/arrow"
            fillMode: Image.Pad // For not stretching image
            anchors.verticalCenter: parent.verticalCenter
            anchors.right: parent.right
            anchors.rightMargin: 12
//            visible: myGroup == "Gobot"
            visible: true
        }
        onHoveredChanged: if (visible) { currentMenuIndex = 0 } /// desktop
//        onClicked: if (visible) { currentMenuIndex = 0} /// android

        RobotListInPopup {
            id: robotListInPopup
            x: assignPointTo.width
            visible: menu.currentMenuIndex === 0
            robotModel: menu.robotModel
            onRobotSelected: {
                robotModel.savePlaceSignal(ip, namePoint, posX, posY, orientation, home)
                currentMenuIndex = -1;
                menu.currentMenuIndex = -1;
                menu.close();
            }
        }
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    property string namePoint: ""

    PopupMenuItem {
        labelText: { namePoint = name
            langue == "English" ? "修改目标点" : "Edit Point"
        }
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onHoveredChanged: if (visible) { currentMenuIndex = 1}
        onTriggered: {
            editPoint()
        }
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    PopupMenuItem {
        labelText: langue == "English" ? "Hide point" : "Hide point"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onHoveredChanged: if (visible) { currentMenuIndex = 2}
        onTriggered: pointModel.hideShowPoint(groupName, name)
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    PopupMenuItem {
        labelText: langue == "English" ? "删除目标点" : "Delete Point"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onHoveredChanged: if (visible) { currentMenuIndex = 3}
        onTriggered: {
            console.log("deleting the point in editpointpopupmapview.qml");
            deletePoint(name)
        }
    }
}
