import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom"
import "../../Model/Path"

Page {
    id: page
    anchors.fill: parent
    property Paths pathModel
    property Paths tmpPathModel
    signal useTmpPathModel(bool use)
    signal closeMenu()


    Frame {
        id: pathMenuFrame
        visible: !createPathMenuFrame.visible && !createGroupMenuFrame.visible
        anchors.fill: parent
        padding: 0
        PathMenuHeader {
            id: pathMenuHeader
            onOpenCreatePathMenu: createPathMenuFrame.visible = true;
            onOpenCreateGroupMenu: createGroupMenuFrame.visible = true;
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
                createGroupMenuFrame.visible = true;
            }
            onEditPath: {
                createPathMenuContent.oldName = name;
                createPathMenuContent.oldGroup = groupName;
                createPathMenuFrame.visible = true;
            }
        }
    }
    Frame {
        id: createPathMenuFrame
        visible: false
        anchors.fill: parent
        padding: 0

        CreateMenuHeader {
            id: createPathMenuHeader
            onBackToMenu: createPathMenuFrame.visible = false;
            txt: "Path"
        }

        CreatePathMenuContent {
            id: createPathMenuContent
            pathModel: page.pathModel
            tmpPathModel: page.tmpPathModel
            onUseTmpPathModel: page.useTmpPathModel(use)
            anchors {
                left: parent.left
                top: createPathMenuHeader.bottom
                right: parent.right
                bottom: parent.bottom
            }
            onBackToMenu: createPathMenuFrame.visible = false;
        }
    }

    Frame {
        id: createGroupMenuFrame
        visible: false
        anchors.fill: parent
        padding: 0

        CreateGroupMenuHeader {
            id: createGroupMenuHeader
            onBackToMenu: createGroupMenuFrame.visible = false;
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
                createGroupMenuFrame.visible = false;
            }
        }
    }
}

