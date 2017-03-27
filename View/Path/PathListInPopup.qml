import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Path"

Menu {
    id: selectPathMenu
    padding: 0
    width: 140
    property Paths pathModel
    property int currentMenuIndex: -1
    signal pathSelected(string pathName, string groupName)

    background: Rectangle {
        implicitWidth: parent.width
        implicitHeight: pathModel.count * Style.menuItemHeight
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
            model: pathModel
            MenuItem {
                contentItem: Label {
                    text: qsTr(groupName)
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
                Layout.preferredHeight: Style.menuItemHeight
                Layout.preferredWidth: parent.width
                leftPadding: Style.menuItemLeftPadding

                Image {
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
                        implicitHeight: paths.count * Style.menuItemHeight
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
                            model: paths

                            MenuItem {
                                anchors {
                                    left: parent.left
                                    right: parent.right
                                }
                                leftPadding: Style.menuItemLeftPadding
                                Layout.preferredHeight: Style.menuItemHeight
                                Layout.preferredWidth: parent.width

                                Label {
                                    text: qsTr(pathName)
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

                                onTriggered: selectPathMenu.pathSelected(pathName, groupName)
                            }
                        }
                    }
                }
            }
        }
    }
}
