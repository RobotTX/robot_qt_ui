import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Model/Point"
import "../../Model/Robot"
import "../../Model/Path"
import "../Custom"

Page {
    id: page
    anchors.fill: parent
    property Points pointModel
    property PointView tmpPointView
    property Robots robotModel
    property Paths pathModel
    property string langue

    signal closeMenu()
    signal setMessageTop(int status, string msg)
    signal editPoint(string name, string groupName)

    property int menuIndex: 0

    onVisibleChanged: {
        if(visible)
            menuIndex = 0;
    }

    Frame {
        id: pointMenuFrame
        visible: menuIndex == 0
        anchors.fill: parent
        padding: 0
        PointMenuHeader {
            id: pointMenuHeader
            langue: page.langue
            onOpenCreatePointMenu: menuIndex = 1;
            onOpenCreateGroupMenu: menuIndex = 2;
            onCloseMenu: page.closeMenu()
        }

        PointMenuContent {
            id: pointMenuContent
            objectName: "pointMenuContent"
            pointModel: page.pointModel
            robotModel: page.robotModel
            pathModel: page.pathModel
            langue: page.langue
            anchors {
                left: parent.left
                top: pointMenuHeader.bottom
                right: parent.right
                bottom: parent.bottom
            }
            onRenameGroup: {
                createGroupMenuContent.oldName = name;
                menuIndex = 2;
            }
            onEditPoint: {
                createPointMenuContent.oldName = name;
                createPointMenuContent.oldGroup = groupName;
                // will open the point menu
                menuIndex = 1;
            }
        }
    }

    Frame {
        id: createPointMenuFrame
        visible: menuIndex == 1
        anchors.fill: parent
        padding: 0

        CreateMenuHeader {
            id: createPointMenuHeader
            onBackToMenu: menuIndex = 0;
            langue: page.langue
            txt: langue == "English" ? "目标点" : "Point"
        }

        CreatePointMenuContent {
            id: createPointMenuContent
            pointModel: page.pointModel
            tmpPointView: page.tmpPointView
            robotModel: page.robotModel
            pathModel: page.pathModel
            langue: page.langue
            anchors {
                left: parent.left
                top: createPointMenuHeader.bottom
                right: parent.right
                bottom: parent.bottom
            }
            onBackToMenu: menuIndex = 0;
            onSetMessageTop: page.setMessageTop(status, msg)
        }
    }

    Frame {
        id: createGroupMenuFrame
        visible: menuIndex == 2
        anchors.fill: parent
        padding: 0

        CreateGroupMenuHeader {
            id: createGroupMenuHeader
            langue: page.langue
            onBackToMenu: menuIndex = 0;
        }

        CreateGroupMenuContent {
            id: createGroupMenuContent
            objectName: "createPointGroupMenu"
            langue: page.langue
            anchors {
                left: parent.left
                top: createGroupMenuHeader.bottom
                right: parent.right
                bottom: parent.bottom
            }
            onBackToMenu: {
                oldName = "";
                menuIndex = 0;
            }
            onSetMessageTop: page.setMessageTop(status, msg)
        }
    }

    Connections {
        target: pointModel
        onSaveCurrentHome: {
            createPointMenuContent.homeX = homeX;
            createPointMenuContent.homeY = homeY;
            createPointMenuContent.homeOri = homeOri;
            createPointMenuContent.homeName = "CS";
            page.menuIndex = 1;
        }
    }

    Connections {
        target: pointModel
        onEditPointB: {
            console.log("we are in pointMenu.qml connections");
            createPointMenuContent.oldName = name;
            createPointMenuContent.oldGroup = groupName;
            page.menuIndex = 1;
            console.log("createPointMenuContent.oldName = " + createPointMenuContent.oldName);
            console.log("createPointMenuContent.oldGroup = " + createPointMenuContent.oldGroup)

        }
    }

    function doubleClickedOnMap(mouseX, mouseY){
        if(!createPointMenuFrame.visible){
            menuIndex = 1;
            tmpPointView.x = mouseX;
            tmpPointView.y = mouseY;
        }
    }



}


