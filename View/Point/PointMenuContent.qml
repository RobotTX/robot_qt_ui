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
        /// Only the invisible "No Group" left and it's empty
        visible: (pointModel.count === 1 && pointModel.get(0).points.count === 0) || pointModel.count === 0
        // TODO double click needs to be implemented
        txt: "You don't have any points yet, click the '+' button or double click the map to create a point."
        imgSrc: "qrc:/icons/big_point"
    }

    Component {
        id: delegate
        PointListItem {
            column: columnId
            width: pointMenuFrame.width
            pointModel: pointMenuFrame.pointModel
            onRenameGroup: pointMenuFrame.renameGroup(name)
            onEditPoint: pointMenuFrame.editPoint(name, groupName)
        }
    }


    Flickable {
        ScrollBar.vertical: ScrollBar { }
        contentHeight: contentItem.childrenRect.height
        anchors.fill: parent
        anchors.topMargin: 10

        Column {
            id: columnId
            property string selectedGroup: Helper.noGroup
            property string selectedPoint: (pointModel.count > 0) ? pointModel.get(0).points.count > 0 ? pointModel.get(0).points.get(0).name : "" : ""
            /// The list containing both the graphical and model of the points in the menu
            Repeater {
                model: pointModel
                delegate: delegate
            }
        }
    }
}
