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
    signal trackObjectSignal(string ip, double x, double y, double orientation)
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
        labelText: langue == "English" ? "Send Point To" : "发送目标点给"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
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
        onHoveredChanged: {
            if (visible) {
                robotListInPopup.open();
                currentMenuIndex = 0;
            } /// desktop
        }
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
            langue == "English" ? "Edit Point" : "修改目标点"
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
        labelText: langue == "English" ? "Hide Point" : "隐藏目标点"
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
        labelText: langue == "English" ? "Delete Point" : "删除目标点"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onHoveredChanged: if (visible) { currentMenuIndex = 3}
        onTriggered: {
            deletePoint(name)
        }
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    PopupMenuItem {
        id: trackObject
        labelText: langue == "English" ? "Track Object" : "寻找目标"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
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
        onHoveredChanged: {
            if (visible) {
                robotListInPopupBis.open();
                currentMenuIndex = 4;
            } /// desktop
        }
//        onClicked: if (visible) { currentMenuIndex = 0} /// android

        RobotListInPopup {
            id: robotListInPopupBis
            x: trackObject.width
            visible: menu.currentMenuIndex === 4
            robotModel: menu.robotModel
            onRobotSelected: {
                robotModel.trackObjectSignal(ip, posX, posY, orientation)
                currentMenuIndex = -1;
                menu.currentMenuIndex = -1;
                menu.close();
            }
        }
    }
}
