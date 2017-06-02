import QtQuick 2.7
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import QtQuick.Extras 1.4
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../../Model/Path"
import "../Point"
import "../Path"
import "../Custom"

Menu {
    id: robotMenu
    padding: 0
    width: 220
    property Points pointModel
    property Paths pathModel
    property int currentMenuIndex: -1
    signal pointSelected(double _homeX, double _homeY, int orientation)
    signal pathSelected(string _pathName, string _groupName)
    signal renameRobot()
    signal deletePath()
    signal laserPressed()

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    PopupMenuItem {
        id: assignHome
        height: Style.menuItemHeight
        width: parent.width
        labelText: "Assign a Charging Station"
        leftPadding: Style.menuItemLeftPadding

        Image {
            asynchronous: true
            source: "qrc:/icons/arrow"
            fillMode: Image.Pad // For not stretching image
            anchors.verticalCenter: parent.verticalCenter
            anchors.right: parent.right
            anchors.rightMargin: 12
        }
        onHoveredChanged: if(visible) currentMenuIndex = 0

        PointListInPopup {
            x: assignHome.width
            visible: robotMenu.currentMenuIndex === 0
            onVisibleChanged: if(!visible) currentMenuIndex = -1
            pointModel: robotMenu.pointModel
            homeOnly: true
            onPointSelected: {
                console.log("robotMenu.pointSelected");
                robotMenu.pointSelected(posX, posY, orientation)
                currentMenuIndex = -1;
                robotMenu.currentMenuIndex = -1;
                robotMenu.close();
            }
        }
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    PopupMenuItem {
        id: assignPath
        height: Style.menuItemHeight
        width: parent.width
        labelText: "Assign a Path"
        leftPadding: Style.menuItemLeftPadding

        Image {
            asynchronous: true
            source: "qrc:/icons/arrow"
            fillMode: Image.Pad // For not stretching image
            anchors.verticalCenter: parent.verticalCenter
            anchors.right: parent.right
            anchors.rightMargin: 12
        }
        onHoveredChanged: if(visible){ currentMenuIndex = 1 }

        PathListInPopup {
            x: assignPath.width
            visible: robotMenu.currentMenuIndex === 1
            onVisibleChanged: if(!visible) currentMenuIndex = -1
            pathModel: robotMenu.pathModel
            onPathSelected: {
                robotMenu.pathSelected(pathName, groupName);
                currentMenuIndex = -1;
                robotMenu.currentMenuIndex = -1;
                robotMenu.close();
            }
        }
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    PopupMenuItem {
        height: Style.menuItemHeight
        width: parent.width
        labelText: laserActivated ? "Hide laser feedback" : "Show laser feedback"
        leftPadding: Style.menuItemLeftPadding

        onHoveredChanged: if(visible) currentMenuIndex = 2
        onTriggered: robotMenu.laserPressed()
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    PopupMenuItem {
        height: Style.menuItemHeight
        width: parent.width
        labelText: "Rename"
        leftPadding: Style.menuItemLeftPadding

        onHoveredChanged: if(visible) currentMenuIndex = 3
        onTriggered: robotMenu.renameRobot()
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    PopupMenuItem {
        height: Style.menuItemHeight
        width: parent.width
        labelText: "Delete Path from Robot"
        leftPadding: Style.menuItemLeftPadding

        onHoveredChanged: if(visible) currentMenuIndex = 4
        onTriggered: robotMenu.deletePath()
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }
}
