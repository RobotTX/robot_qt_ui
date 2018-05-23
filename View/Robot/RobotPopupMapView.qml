import QtQuick 2.7
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import QtQuick.Extras 1.4
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../../Model/Path"
import "../../Model/Robot"
import "../Point"
import "../Path"
import "../Custom"

Menu {
    id: robotMenu
    padding: 0
    width: 220
    property Points pointModel
    property Paths pathModel
    property Robots robotModel
    property string langue
    property int currentMenuIndex: -1
    signal pointSelected(double _homeX, double _homeY, int orientation)
    signal pathSelected(string _pathName, string _groupName)
    signal renameRobot()
    signal deletePath()
    signal laserPressed()
    signal saveCurrentPath()
    signal saveCurrentHome()
    signal soundOn(string ip)
    signal soundOff(string ip)

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
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
        labelText: langue == "English" ? "设置路径" : "Assign Path"
        leftPadding: Style.menuItemLeftPadding

        Image {
            asynchronous: true
            source: "qrc:/icons/arrow"
            fillMode: Image.Pad // For not stretching image
            anchors.verticalCenter: parent.verticalCenter
            anchors.right: parent.right
            anchors.rightMargin: 12
        }
        onHoveredChanged: if(visible){ currentMenuIndex = 1 } /// desktop
//        onClicked: if(visible) currentMenuIndex = 1 /// android

        PathListInPopup {
            x: assignPath.width
            visible: robotMenu.currentMenuIndex === 1
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

//    PopupMenuItem {
//        height: Style.menuItemHeight
//        width: parent.width
//        labelText: laserActivated ? "Hide laser feedback" : "Show laser feedback"
//        leftPadding: Style.menuItemLeftPadding

//        onHoveredChanged: if(visible) currentMenuIndex = 2
//        onTriggered: robotMenu.laserPressed()
//    }

    PopupMenuItem {
        height: Style.menuItemHeight
        width: parent.width
        labelText: langue == "English" ? "保存当前路径" : "Save My path"
        leftPadding: Style.menuItemLeftPadding

//        onHoveredChanged: if(visible) currentMenuIndex = 3
        onClicked: if(visible) currentMenuIndex = 3
        onTriggered: {
            robotMenu.saveCurrentPath();
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
        labelText: langue == "English" ? "删除当前路径" : "Delete My Path"
        leftPadding: Style.menuItemLeftPadding

//        onHoveredChanged: if(visible) currentMenuIndex = 5
        onClicked: if(visible) currentMenuIndex = 5
        onTriggered: robotMenu.deletePath()
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    PopupMenuItem {
        height: Style.menuItemHeight
        width: parent.width
//        labelText: langue == "English" ? "保存当前路径" : "Mute"
        labelText: charging !== true ? "Mute" : "Unmute"
        leftPadding: Style.menuItemLeftPadding
        onTriggered: {
            charging = !charging;
            if (charging !== true) {
                robotModel.soundOn(ip)
            } else {
                // if mute is true then the corresponding command is x
                robotModel.soundOff(ip);
            }
        }
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }
}
