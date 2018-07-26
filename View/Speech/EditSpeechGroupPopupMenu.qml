import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../Custom"

Menu {
    padding: 0
    width: 151
    signal renameGroup()
    signal deleteGroup()

    property string langue

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    PopupMenuItem {
        labelText: langue == "English" ? "Rename Group" : "重命名组"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: renameGroup() // called here
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    PopupMenuItem {
        labelText: langue == "English" ? "Delete Group" : "删除组"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: deleteGroup() // called here
    }
}

