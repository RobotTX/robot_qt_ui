import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../Custom"

Frame {
    id: pointMenuFrame
    padding: 0
    property Points pointModel

    signal deletePoint(string name, string groupName)
    signal deleteGroup(string name)
    signal renameGroup(string name)
    signal moveTo(string name, string oldGroup, string newGroup)
    signal editPoint(string name, string groupName)

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    /// This frame is displayed when there is no point
    EmptyMenu {
        /// Only the invisible "No Group" left
        visible: pointModel.count === 1 && pointModel.get(0).points.count === 0
        txt: "You have no points, click the '+' button or double click the map to create a point."
        imgSrc: "qrc:/icons/big_point"
    }

    /// This frame is displayed when we have points
    Component {
        id: delegate
        PointListItem {
            column: columnId
            width: pointMenuFrame.width
            pointModel: pointMenuFrame.pointModel
            onDeleteGroup: pointModel.deleteGroup(name)
            onRenameGroup: pointMenuFrame.renameGroup(name)
            onDeletePoint: pointModel.deletePoint(groupName, name)
            onMoveTo: pointModel.moveTo(name, oldGroup, newGroup)
            onEditPoint: pointMenuFrame.editPoint(name, groupName)
            onHideShowGroup: pointModel.hideShowGroup(groupName)
            onHideShowPoint: pointModel.hideShowPoint(groupName, name)
        }
    }

    Column {
        id: columnId
        property string selectedGroup: Helper.noGroup
        property string selectedPoint: (pointModel.count > 0) ? pointModel.get(0).points.count > 0 ? pointModel.get(0).points.get(0).name : "" : ""
        //anchors.fill: parent
        /// The list containing both the graphical and model of the points in the menu
        Repeater {
            id: pointList
            objectName: "pointList"
            anchors.fill: parent
            model: pointModel
            delegate: delegate
            focus: true
            anchors.topMargin: 14
        }
    }
}
