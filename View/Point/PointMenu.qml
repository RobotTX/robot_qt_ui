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

    onVisibleChanged: {
        createPointMenuFrame.visible = false;
    }

    Frame {
        id: pointMenuFrame
        visible: !createPointMenuFrame.visible && !createGroupMenuFrame.visible
        anchors.fill: parent
        padding: 0
        PointMenuHeader {
            id: pointMenuHeader
            onOpenCreatePointMenu: createPointMenuFrame.visible = true;
            onOpenCreateGroupMenu: createGroupMenuFrame.visible = true;
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
                createGroupMenuFrame.visible = true;
            }
            onEditPoint: {
                createPointMenuContent.oldName = name;
                createPointMenuContent.oldGroup = groupName;
                createPointMenuFrame.visible = true;
            }
        }
    }

    Frame {
        id: createPointMenuFrame
        visible: false
        anchors.fill: parent
        padding: 0

        CreateMenuHeader {
            id: createPointMenuHeader
            onBackToMenu: createPointMenuFrame.visible = false;
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
            onBackToMenu: createPointMenuFrame.visible = false;
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
            objectName: "createPointGroupMenu"
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


