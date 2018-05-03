import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Path"
import "../../Model/Robot"
import "../Custom"

Column {
    id: groupListItem

    property Paths pathModel
    property Column column
    property Robots robotModel
    property string langue
    property string groupSelected

    signal groupSelected(string groupName)

    Repeater {
        model: pathModel.pathsGuide.length
        delegate: delegate
    }

    Component {
        id: delegate

        Rectangle {
            color: "yellow"
            width: 10
            height: 10
        }
    }
}

