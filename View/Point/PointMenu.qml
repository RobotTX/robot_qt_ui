import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Model/Point"

Page {
    id: page
    readonly property int index: 2
    anchors.fill: parent
    property Points _pointModel;

    Frame {
        id: pointMenuFrame
        visible: !createPointMenuFrame.visible
        anchors.fill: parent
        padding: 0
        PointMenuHeader {
            id: pointMenuHeader
            objectName: "pointMenuHeader"
            txt: "Point"
            onOpenCreatePointMenu: createPointMenuFrame.visible = true;
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
        }
    }

    Frame {
        id: createPointMenuFrame
        visible: false
        anchors.fill: parent
        padding: 0

        CreatePointMenuHeader {
            id: createPointMenuHeader
            onBackToMenu: createPointMenuFrame.visible = false;
        }

        CreatePointMenuContent {
            pointModel: _pointModel
            anchors {
                left: parent.left
                top: createPointMenuHeader.bottom
                right: parent.right
                bottom: parent.bottom
            }
        }
    }
}


