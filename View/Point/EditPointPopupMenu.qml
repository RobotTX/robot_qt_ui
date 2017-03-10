import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Menu {
    padding: 0
    width: 188
    signal editPoint()
    signal renamePoint()
    signal deletePoint()
    signal moveTo(string groupName)

    background: Rectangle {
        implicitWidth: parent.width
        implicitHeight: 4 * Style.menuItemHeight + 6
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    MenuItem {
        text: qsTr("Edit Point")
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: editPoint()
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 2
    }

    MenuItem {
        text: qsTr("Rename Point")
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: renamePoint()
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 2
    }

    MenuItem {
        text: qsTr("Move to")
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        //onTriggered: openCreateGroupMenu()

    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 2
    }

    MenuItem {
        text: qsTr("Delete Point")
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        onTriggered: deletePoint()
    }
}
