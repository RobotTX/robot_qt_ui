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
    signal pointSelected(string name, double posX, double posY)

    background: Rectangle {
        implicitWidth: parent.width
        implicitHeight: pointModel.count * Style.menuItemHeight
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }

    ColumnLayout {
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
                Layout.preferredHeight: Style.menuItemHeight
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
                    visible: currentMenuIndex === index


                    background: Rectangle {
                        implicitWidth: parent.width
                        implicitHeight: points.count * Style.menuItemHeight
                        color: Style.lightGreyBackground
                        border.color: Style.lightGreyBorder
                        radius: 5
                    }


                    ColumnLayout {
                        anchors {
                            left: parent.left
                            right: parent.right
                        }

                        Repeater {
                            model: points

                            PopupMenuItem {
                                anchors {
                                    left: parent.left
                                    right: parent.right
                                }
                                leftPadding: Style.menuItemLeftPadding
                                Layout.preferredHeight: Style.menuItemHeight
                                Layout.preferredWidth: parent.width

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

                                onTriggered: selectPointMenu.pointSelected(name, posX, posY)
                            }
                        }
                    }
                }
            }
        }
    }
}
