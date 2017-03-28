import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../Custom"

Menu {
    padding: 0
    width: 151
    signal renameGroup()
    signal deleteGroup()

    background: Rectangle {
        implicitWidth: parent.width
        implicitHeight: 2 * Style.menuItemHeight + 2
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    PopupMenuItem {
        text: qsTr("Rename Group")
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: renameGroup()
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 2
    }

    PopupMenuItem {
        text: qsTr("Delete Group")
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: deleteGroup()
    }
}
