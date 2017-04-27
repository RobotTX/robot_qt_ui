import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Path"
import "../Custom"

Menu {
    id: selectPathMenu
    padding: 0
    width: 140
    property Paths pathModel
    property int currentMenuIndex: -1
    signal pathSelected(string pathName, string groupName)

    background: Rectangle {
        implicitWidth: parent.width
        implicitHeight: pathModel.count * (1 + Style.menuItemHeight) - 1
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
            model: pathModel
            delegate: PopupMenuItem {
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
                Layout.preferredHeight: 1 + Style.menuItemHeight
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
                onHoveredChanged: if(visible && !pathMenu.visible) currentMenuIndex = index

                Menu {
                    id: pathMenu
                    padding: 0
                    width: 140
                    x: parent.width
                    visible: currentMenuIndex === index

                    background: Rectangle {
                        implicitWidth: parent.width
                        implicitHeight: paths.count * (1 + Style.menuItemHeight) - 1
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
                            model: paths
                            delegate: PopupMenuItem {
                                anchors {
                                    left: parent.left
                                    right: parent.right
                                }
                                leftPadding: Style.menuItemLeftPadding
                                Layout.preferredHeight: 1 + Style.menuItemHeight
                                Layout.preferredWidth: parent.width

                                contentItem: CustomLabel {
                                    text: qsTr(pathName)
                                    anchors {
                                        left: parent.left
                                        right: parent.right
                                        leftMargin: 20
                                        rightMargin: 5
                                        verticalCenter: parent.verticalCenter
                                    }
                                }

                                onTriggered: selectPathMenu.pathSelected(pathName, groupName)
                            }
                        }
                    }
                }
            }
        }
    }
}
