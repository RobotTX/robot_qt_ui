import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../Custom"

Menu {
    id: selectPointMenu
    padding: 0
    width: 140
    property Points pointModel
    property int currentMenuIndex: -1
    property bool homeOnly: false
    signal pointSelected(string name, double posX, double posY, int orientation)

    background: Rectangle {
        implicitWidth: parent.width
        // +1 is needed to compensate the line that we don't have a line in these menus
        // but we have it in the previous menu
        implicitHeight: pointModel.count * (Style.menuItemHeight+1) - 1
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    ColumnLayout {
        spacing: 0
        anchors {
            left: parent.left
            right: parent.right
        }

        Repeater {
            model: pointModel
            PopupMenuItem {
                contentItem: CustomLabel {
                    text: qsTr(groupName)
                    anchors {
                        left: parent.left
                        right: arrow.left
                        leftMargin: 20
                        rightMargin: 5
                        verticalCenter: parent.verticalCenter
                    }
                }
                Layout.preferredHeight: Style.menuItemHeight+1
                Layout.preferredWidth: parent.width
                leftPadding: Style.menuItemLeftPadding

                Image {
                    id: arrow
                    asynchronous: true
                    source: "qrc:/icons/arrow"
                    fillMode: Image.Pad // For not stretching image
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.right: parent.right
                    anchors.rightMargin: 12
                }
                onHoveredChanged: if(visible && !pointMenu.visible) currentMenuIndex = index

                Menu {
                    id: pointMenu
                    padding: 0
                    width: 140
                    x: parent.width
                    visible: currentMenuIndex === index && (homeOnly ? pointModel.getNbHome(groupName) > 0 : true)

                    background: Rectangle {
                        implicitWidth: parent.width
                        implicitHeight: (homeOnly ? pointModel.getNbHome(groupName) : points.count) * (Style.menuItemHeight+1)-1
                        color: Style.lightGreyBackground
                        border.color: Style.lightGreyBorder
                        radius: 5
                    }


                    ColumnLayout {
                        spacing: 0
                        anchors {
                            left: parent.left
                            right: parent.right
                        }

                        Repeater {
                            model: points

                            PopupMenuItem {
                                leftPadding: Style.menuItemLeftPadding
                                Layout.preferredHeight: homeOnly ? (home ? Style.menuItemHeight+1 : 0) : Style.menuItemHeight+1
                                Layout.preferredWidth: parent.width
                                visible: homeOnly ? home : true

                                contentItem: CustomLabel {
                                    text: qsTr(name)
                                    anchors {
                                        left: parent.left
                                        right: parent.right
                                        leftMargin: 20
                                        rightMargin: 5
                                        verticalCenter: parent.verticalCenter
                                    }
                                }

                                onTriggered: selectPointMenu.pointSelected(name, posX, posY, orientation)
                            }
                        }
                    }
                }
            }
        }
    }
}
