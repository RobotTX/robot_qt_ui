import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom"
import "../../Model/Path"
import "../../Model/Point"
import "../../Model/Robot"
import "../../Helper/helper.js" as Helper

Page {
    id: page
    anchors.fill: parent
    property Paths pathModel // Path file in qml model
    property Paths tmpPathModel
    property Points pointModel
    property Robots robotModel
    property string langue
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
            langue: page.langue
            onOpenCreatePathMenu: menuIndex = 1;
            onOpenCreateGroupMenu: menuIndex = 2;
            onCloseMenu: page.closeMenu()
        }

        PathMenuContent {
            pathModel: page.pathModel
            robotModel: page.robotModel
            langue: page.langue
            anchors {
                left: parent.left
                top: pathMenuHeader.bottom
                right: parent.right
                bottom: parent.bottom
            }
            onRenameGroup: {
                console.log("langue = " + langue)
                createGroupMenuContent.oldName = name;
                menuIndex = 2;
            }
            onEditPath: {
                console.log("langue = " + langue)
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
            langue: page.langue
            onBackToMenu: menuIndex = 0;
            txt: "Path"
        }

        CreatePathMenuContent {
            id: createPathMenuContent
            pathModel: page.pathModel
            tmpPathModel: page.tmpPathModel
            pointModel: page.pointModel
            langue: page.langue
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
            langue: page.langue
            onBackToMenu: menuIndex = 0;
        }

        CreateGroupMenuContent {
            id: createGroupMenuContent
            objectName: "createPathGroupMenu"
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
        target: pathModel
        onSaveCurrentPath: {
            console.log("We are in PathMenu - onSaveCurrentPath");
            console.log("pathname = " + pathName + " no pathPoints = " + pathPoints.count)
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
