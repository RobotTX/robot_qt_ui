import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../../Model/Path"
import "../Point"
import "../Path"

Menu {
    id: robotMenu
    padding: 0
    width: 190
    property Points pointModel
    property Paths pathModel
    property int currentMenuIndex: -1
    signal pointSelected(string name, double posX, double posY)

    background: Rectangle {
        implicitWidth: parent.width
        implicitHeight: 110
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    MenuItem {
        id: assignHome
        height: Style.menuItemHeight
        width: parent.width
        contentItem: Label {
            text: qsTr("Assign a Home Point")
            anchors {
                left: parent.left
                right: parent.right
                leftMargin: 20
                rightMargin: 5
                verticalCenter: parent.verticalCenter
            }
            maximumLineCount: 1
            elide: Text.ElideRight
        }
        leftPadding: Style.menuItemLeftPadding

        Image {
            asynchronous: true
            source: "qrc:/icons/arrow"
            fillMode: Image.Pad // For not stretching image
            anchors.verticalCenter: parent.verticalCenter
            anchors.right: parent.right
            anchors.rightMargin: 12
        }
        onHoveredChanged: if(visible) currentMenuIndex = 0

        PointListInPopup {
            x: assignHome.width
            visible: robotMenu.currentMenuIndex === 0
            onVisibleChanged: if(!visible) currentMenuIndex = -1
            pointModel: robotMenu.pointModel
        }
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 2
    }

    MenuItem {
        id: assignPath
        height: Style.menuItemHeight
        width: parent.width
        contentItem: Label {
            text: qsTr("Assign a Path")
            anchors {
                left: parent.left
                right: parent.right
                leftMargin: 20
                rightMargin: 5
                verticalCenter: parent.verticalCenter
            }
            maximumLineCount: 1
            elide: Text.ElideRight
        }
        leftPadding: Style.menuItemLeftPadding

        Image {
            asynchronous: true
            source: "qrc:/icons/arrow"
            fillMode: Image.Pad // For not stretching image
            anchors.verticalCenter: parent.verticalCenter
            anchors.right: parent.right
            anchors.rightMargin: 12
        }
        onHoveredChanged: if(visible) currentMenuIndex = 1

        PathListInPopup {
            x: assignPath.width
            visible: robotMenu.currentMenuIndex === 1
            onVisibleChanged: if(!visible) currentMenuIndex = -1
            pathModel: robotMenu.pathModel
        }
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 2
    }

    MenuItem {
        height: Style.menuItemHeight
        width: parent.width
        contentItem: Label {
            text: qsTr("Rename")
            anchors {
                left: parent.left
                right: parent.right
                leftMargin: 20
                rightMargin: 5
                verticalCenter: parent.verticalCenter
            }
            maximumLineCount: 1
            elide: Text.ElideRight
        }
        leftPadding: Style.menuItemLeftPadding

        onHoveredChanged: if(visible) currentMenuIndex = 2
    }

    Rectangle {
        color: Style.lightGreyBorder
        width: parent.width
        height: 2
    }

    MenuItem {
        height: Style.menuItemHeight
        width: parent.width
        contentItem: Label {
            text: qsTr("Delete Path from Robot")
            anchors {
                left: parent.left
                right: parent.right
                leftMargin: 20
                rightMargin: 5
                verticalCenter: parent.verticalCenter
            }
            maximumLineCount: 1
            elide: Text.ElideRight
        }
        leftPadding: Style.menuItemLeftPadding

        onHoveredChanged: if(visible) currentMenuIndex = 3
    }
}
