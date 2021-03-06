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
    property int menuIndex: -1
    property bool homeOnly: false
    signal pointSelected(string name, double posX, double posY, int orientation)

    background: Rectangle {
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
            delegate: PopupMenuItem {
                labelText: groupName
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
                onHoveredChanged: {
                    if (visible) {
                        pointMenu.open();
                        currentMenuIndex = index;
                    } /// desktop
                }
//                onClicked: if(visible && !pointMenu.visible) currentMenuIndex = index /// android


                Menu {
                    id: pointMenu
                    padding: 0
                    width: 140
                    x: parent.width
                    visible: (currentMenuIndex === index && menuIndex === 0)
//                    visible: currentMenuIndex === index
                    background: Rectangle {
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
                            delegate: PopupMenuItem {
                                leftPadding: Style.menuItemLeftPadding
                                Layout.preferredHeight: homeOnly ? (home ? Style.menuItemHeight+1 : 0) : Style.menuItemHeight+1
                                Layout.preferredWidth: parent.width
                                visible: homeOnly ? home : true
                                labelText: name

                                onTriggered: {
                                    selectPointMenu.pointSelected(name, posX, posY, orientation)
                                    pointMenu.close();
                                    selectPointMenu.close();
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
