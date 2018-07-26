import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../Custom"

Menu {
    padding: 0
    width: 117
    signal openCreatePathMenu()
    signal openCreateGroupMenu()

    property string langue

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    PopupMenuItem {
        labelText: langue == "English" ? "New Path" : "创建路径"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: openCreatePathMenu()
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 1
    }

    PopupMenuItem {
        labelText: langue == "English" ? "New Group" : "创建组"
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: openCreateGroupMenu()
    }
}
