import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../Custom"
import "../../Model/Path"
import "../../Model/Robot"

Frame {
    id: pathMenuFrame
    padding: 0
    signal renameGroup(string name)
    signal editPath(string name, string groupName)

    property Paths pathModel
    property Robots robotModel
    property string langue
    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    /// This frame is displayed when there is no path
    EmptyMenu {
        visible: (pathModel.count === 1 && pathModel.get(0).paths.count === 0) || pathModel.count === 0
        txt: langue == "English" ? "No path created.\nPelase click + button to create a path." : "没有任何路径.\n请点击 + 按钮，创建路径"
        imgSrc: "qrc:/icons/big_path"
        font.pointSize: Style.ubuntuSubHeadingSize
    }

    Component {
        id: delegate
        PathListItem {
            column: columnId
            width: pathMenuFrame.width
            pathModel: pathMenuFrame.pathModel
            robotModel: pathMenuFrame.robotModel
            langue: pathMenuFrame.langue
            onRenameGroup: pathMenuFrame.renameGroup(name)
            onEditPath: pathMenuFrame.editPath(name, groupName)
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
            property string selectedPath: (pathModel.count > 0) ? pathModel.get(0).paths.count > 0 ? pathModel.get(0).paths.get(0).pathName : "" : ""
            /// The list containing both the graphical and model of the paths in the menu
            Repeater {
                model: pathModel
                delegate: delegate
            }
        }
    }
}
