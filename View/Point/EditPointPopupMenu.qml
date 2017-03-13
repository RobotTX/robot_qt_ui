import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"

Menu {
    id: menu
    padding: 0
    width: 188

    property Points pointModel
    property string myGroup

    signal editPoint()
    signal renamePoint()
    signal deletePoint()
    signal moveTo(string groupName)

    background: Rectangle {
        implicitWidth: parent.width
        implicitHeight: 3 * Style.menuItemHeight + 4
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
        text: qsTr("Move to")
        width: parent.width
        leftPadding: Style.menuItemLeftPadding
        height: Style.menuItemHeight
        //onTriggered: openCreateGroupMenu()

        Image {
            asynchronous: true
            source: "qrc:/icons/arrow"
            fillMode: Image.Pad // For not stretching image
            anchors.verticalCenter: parent.verticalCenter
            anchors.right: parent.right
            anchors.rightMargin: 12
        }
        onHoveredChanged: if(visible && !moveToMenu.visible) moveToMenu.open()


        Menu {
            id: moveToMenu
            padding: 0
            width: 140
            x: parent.width
            property int nbGroup: Helper.getGroupList(menu.pointModel).length

            background: Rectangle {
                implicitWidth: parent.width
                implicitHeight: (moveToMenu.nbGroup + 1) * Style.menuItemHeight + 2
                color: Style.lightGreyBackground
                border.color: Style.lightGreyBorder
                radius: 5
            }

            MenuItem {
                text: qsTr("New Group")
                width: parent.width
                leftPadding: Style.menuItemLeftPadding
                height: Style.menuItemHeight
            }

            Rectangle {
                color: Style.lightGreyBorder
                width: parent.width
                height: 2
            }

            ColumnLayout {

                Repeater {
                    model: Helper.getGroupList(menu.pointModel)

                    MenuItem {
                        Layout.preferredHeight: Style.menuItemHeight
                        text: qsTr(modelData)
                        width: parent.width
                        leftPadding: Style.menuItemLeftPadding
                        /// Disable the group in which the point already is so we can't move it in
                        enabled: !(modelData === myGroup)
                        onTriggered: moveTo(modelData)
                    }
                }
            }
        }
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
