import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom"
import "../../Model/Path"
import "../../Model/Point"
import "../../Helper/helper.js" as Helper

Page {
    id: page
    anchors.fill: parent
    property Paths pathModel // Path file in qml model
    property Paths tmpPathModel
    property Points pointModel
    property int menuIndex: 0
    signal useTmpPathModel(bool use)
    signal useRobotPathModel(bool use)
    signal closeMenu()
    signal setMessageTop(int status, string msg)

    onVisibleChanged: {
        if(visible){
            useRobotPathModel(false);
            pathModel.visiblePathChanged();
            menuIndex = 0;
        }
    }

    Frame {
        id: pathMenuFrame
        visible: menuIndex == 0
        anchors.fill: parent
        padding: 0
        PathMenuHeader {
            id: pathMenuHeader
            onOpenCreatePathMenu: menuIndex = 1;
            onOpenCreateGroupMenu: menuIndex = 2;
            onCloseMenu: page.closeMenu()
        }

        PathMenuContent {
            pathModel: page.pathModel
            anchors {
                left: parent.left
                top: pathMenuHeader.bottom
                right: parent.right
                bottom: parent.bottom
            }
            onRenameGroup: {
                createGroupMenuContent.oldName = name;
                menuIndex = 2;
            }
            onEditPath: {
                createPathMenuContent.oldName = name;
                createPathMenuContent.oldGroup = groupName;
                menuIndex = 1;
            }
        }
    }
    Frame {
        id: createPathMenuFrame
        visible: menuIndex == 1
        anchors.fill: parent
        padding: 0

        CreateMenuHeader {
            id: createPathMenuHeader
            onBackToMenu: menuIndex = 0;
            txt: "Path"
        }

        CreatePathMenuContent {
            id: createPathMenuContent
            pathModel: page.pathModel
            tmpPathModel: page.tmpPathModel
            pointModel: page.pointModel
            onUseTmpPathModel: page.useTmpPathModel(use)
            anchors {
                left: parent.left
                top: createPathMenuHeader.bottom
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
            onBackToMenu: menuIndex = 0;
        }

        CreateGroupMenuContent {
            id: createGroupMenuContent
            objectName: "createPathGroupMenu"
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
        target: pathModel
        onSaveCurrentPath: {
            console.log("pathname = " + pathName + " no pathPoints = " + pathPoints.count + " waitTime " + waitTime)
            createPathMenuContent.robotPathName = pathName;
            createPathMenuContent.robotPathPoints = pathPoints;

            page.menuIndex = 1;
            console.log("menu = " + page.menuIndex);
        }

    }

//    function saveCurrentPath(pathName, pathPoints) {
//        console.log("pathname = " + pathName + " no pathPoints = " + pathPoints.count)
//    }
}

