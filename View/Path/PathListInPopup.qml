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
    property int menuIndex: -1
    signal pathSelected(string pathName, string groupName)
    signal pathHovered(string pathName, string groupName)

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
                labelText: groupName
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

                onHoveredChanged: {
                    if (visible) {
                        pathMenu.open();
                        currentMenuIndex = index;
                        console.log("currentMenuIndex = " + currentMenuIndex);
                    } /// desktop
                }
//                onClicked: if(visible && !pathMenu.visible) currentMenuIndex = index /// android

                Menu {
                    id: pathMenu
                    padding: 0
                    width: 140
                    x: parent.width
                    visible: (currentMenuIndex === index && menuIndex === 1)
//                    visible: currentMenuIndex === index

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
                                labelText:  pathName
                                onHoveredChanged: {
                                    pathModel.hideShowPathOnMap(groupName, pathName);
                                }

                                onTriggered: {
                                    selectPathMenu.pathSelected(pathName, groupName);
                                    pathMenu.close();
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
