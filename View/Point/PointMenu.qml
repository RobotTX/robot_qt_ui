import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Model/Point"
import "../Custom"

Page {
    id: page
    anchors.fill: parent
    property Points pointModel
    property PointView tmpPointView

    signal closeMenu()
    signal setMessageTop(int status, string msg)

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
            onOpenCreatePointMenu: menuIndex = 1;
            onOpenCreateGroupMenu: menuIndex = 2;
            onCloseMenu: page.closeMenu()
        }

        PointMenuContent {
            id: pointMenuContent
            pointModel: _pointModel
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
            txt: "Point"
        }

        CreatePointMenuContent {
            id: createPointMenuContent
            pointModel: page.pointModel
            tmpPointView: page.tmpPointView
            anchors {
                left: parent.left
                top: createPointMenuHeader.bottom
                right: parent.right
                bottom: parent.bottom
            }
            onBackToMenu: menuIndex = 0;
            onCreatePoint: setMessageTop(2, oldName === "" ? "Created the point \"" + name + "\" in \"" + groupName + "\"" :
                                                           "Edited a point from \"" + oldName + "\" in \"" + oldGroup + "\" to \"" + name + "\" in \"" + groupName + "\"")
        }
    }

    Frame {
        id: createGroupMenuFrame
        visible: menuIndex == 2
        anchors.fill: parent
        padding: 0

        CreateGroupMenuHeader {
            id: createGroupMenuHeader
            onBackToMenu: menuIndex = 0;
        }

        CreateGroupMenuContent {
            id: createGroupMenuContent
            objectName: "createPointGroupMenu"
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

    function doubleClickedOnMap(mouseX, mouseY){
        if(!createPointMenuFrame.visible){
            menuIndex = 1;
            tmpPointView.setPos(mouseX, mouseY);
        }
    }
}


