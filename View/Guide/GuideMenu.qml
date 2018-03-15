import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Model/Point"
import "../../Model/Robot"
import "../../Model/Path"
import "../Point"
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

        GuideMenuHeader {
            id: pointMenuHeader
            langue: page.langue
            onOpenCreateSpeechMenu: menuIndex = 1;
            onOpenCreateGroupMenu: menuIndex = 2;
            onCloseMenu: page.closeMenu()
        }

        GuideMenuContent {
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
            createPointMenuContent.oldName = name;
            createPointMenuContent.oldGroup = groupName;
            page.menuIndex = 1;

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


