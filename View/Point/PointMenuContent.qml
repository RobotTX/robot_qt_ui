import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Model/Point"
import "../Custom"

Frame {
    id: pointMenuFrame
    objectName: "pointMenuFrame"
    padding: 0
    property Points pointModel

    signal deletePoint(string name, string groupName)
    signal deleteGroup(string name)
    signal renameGroup(string name)

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    /// This frame is displayed when there is no point
    EmptyMenu {
        /// Only the invisible "No Group" left
        visible: pointList.count == 1
        txt: "You have no points, click the '+' button or double click the map to create a point."
        imgSrc: "qrc:/icons/big_point"
    }

    /// This frame is displayed when we have points
    Component {
        id: delegate
        PointListItem {
            width: pointMenuFrame.width
            myList: pointList
            onDeleteGroup: pointMenuFrame.deleteGroup(name)
            onRenameGroup: pointMenuFrame.renameGroup(name)
            onDeletePoint: pointMenuFrame.deletePoint(name, groupName)
            pointModel: pointMenuFrame.pointModel
        }
    }

    /// The list containing both the graphical and model of the points in the menu
    ListView {
        id: pointList
        objectName: "pointList"
        anchors.fill: parent
        model: pointModel
        delegate: delegate
        focus: true
        anchors.topMargin: 14

        signal hideShow(string name, string groupName, bool isVisible)
    }
}
