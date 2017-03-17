import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../Custom"
import "../../Model/Path"

Frame {
    id: pathMenuFrame
    padding: 0

    property Paths pathModel
    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    /// This frame is displayed when there is no path
    EmptyMenu {
        visible: (pathModel.count === 1 && pathModel.get(0).paths.count === 0) || pathModel.count === 0
        txt: "You have no path, click the '+' button to create a path."
        imgSrc: "qrc:/icons/big_path"
    }

    Component {
        id: delegate
        PathListItem {
            column: columnId
            width: pathMenuFrame.width
            pathModel: pathMenuFrame.pathModel
            onRenameGroup: pointMenuFrame.renameGroup(name)
            onEditPath: pathMenuFrame.editPath(name, groupName)
        }
    }

    Flickable {
        ScrollBar.vertical: ScrollBar { }
        contentHeight: contentItem.childrenRect.height
        anchors.fill: parent

        Column {
            id: columnId
            property string selectedGroup: Helper.noGroup
            property string selectedPath: (pathModel.count > 0) ? pathModel.get(0).paths.count > 0 ? pathModel.get(0).paths.get(0).pathName : "" : ""
            /// The list containing both the graphical and model of the points in the menu
            Repeater {
                model: pathModel
                delegate: delegate
            }
        }
    }
}
