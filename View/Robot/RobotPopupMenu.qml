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
    property int currentMenuIndex
    signal pointSelected(double _homeX, double _homeY, int orientation)
    signal pathSelected(string _pathName, string _groupName)
    signal renameRobot()
    signal deletePath()
    signal laserPressed()
    signal saveCurrentPath()
    signal saveCurrentHome()
    signal doNothing()

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
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
        onHoveredChanged: if(visible) {
                            pathListInPopup.open();
                            currentMenuIndex = 1 ;
                          } /// desktop
        onTriggered: {
            doNothing();
        }
//        onClicked: if(visible) currentMenuIndex = 1 /// android

        PathListInPopup {
            id: pathListInPopup
            x: assignPath.width
            visible: robotMenu.currentMenuIndex === 1
            pathModel: robotMenu.pathModel
            menuIndex: robotMenu.currentMenuIndex
            onPathSelected: {
                robotMenu.pathSelected(pathName, groupName);
                robotMenu.currentMenuIndex = -1;
                robotModel.assignPathClicked = true;
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
        labelText: langue == "English" ? "保存当前路径" : "Save My Path"
        leftPadding: Style.menuItemLeftPadding

        onHoveredChanged: if(visible) currentMenuIndex = 3
//        onClicked: if(visible) currentMenuIndex = 3
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
        id: assignHome
        height: Style.menuItemHeight
        width: parent.width
        labelText: langue == "English" ? "设置充电站" : "Assign Home"
        leftPadding: Style.menuItemLeftPadding

        Image {
            asynchronous: true
            source: "qrc:/icons/arrow"
            fillMode: Image.Pad // For not stretching image
            anchors.verticalCenter: parent.verticalCenter
            anchors.right: parent.right
            anchors.rightMargin: 12
        }
        onHoveredChanged: if(visible) {
                            pointListInPopup.open();
                            currentMenuIndex = 0;
                          }/// desktop

        onTriggered: {
            doNothing();
        }
//        onClicked: if(visible) currentMenuIndex = 0 /// android

        PointListInPopup {
            id: pointListInPopup
            x: assignHome.width
            visible: robotMenu.currentMenuIndex === 0
            pointModel: robotMenu.pointModel
            homeOnly: true
            menuIndex: robotMenu.currentMenuIndex
            onPointSelected: {
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
        height: Style.menuItemHeight
        width: parent.width
        labelText: langue == "English" ? "保存当前充电站" : "Save My Home"
        leftPadding: Style.menuItemLeftPadding

        onHoveredChanged: if(visible) currentMenuIndex = 2
//        onClicked: if(visible) currentMenuIndex = 2
        onTriggered: robotMenu.saveCurrentHome()
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    PopupMenuItem {
        height: Style.menuItemHeight
        width: parent.width
        labelText: langue == "English" ? "重命名" : "Rename"
        leftPadding: Style.menuItemLeftPadding

        onHoveredChanged: if(visible) currentMenuIndex = 4
//        onClicked: if(visible) currentMenuIndex = 4
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
        labelText: langue == "English" ? "删除当前路径" : "Delete My Path"
        leftPadding: Style.menuItemLeftPadding

        onHoveredChanged: if(visible) currentMenuIndex = 5
//        onClicked: {

//            console.log("button delete path clicked");
//            if(visible) currentMenuIndex = 5
//        }
        onTriggered: {
            robotModel.deletePathButtonClicked = true;
            robotMenu.deletePath()
        }
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }
}
