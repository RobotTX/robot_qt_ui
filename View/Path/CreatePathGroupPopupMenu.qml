import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Menu {
    padding: 0
    width: 117
    signal openCreatePathMenu()
    signal openCreateGroupMenu()

    background: Rectangle {
        implicitWidth: parent.width
        implicitHeight: 2 * Style.menuItemHeight + 2
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    MenuItem {
        text: qsTr("New Path")
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: openCreatePathMenu()
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 2
    }

    MenuItem {
        text: qsTr("New Group")
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: openCreateGroupMenu()
    }
}
