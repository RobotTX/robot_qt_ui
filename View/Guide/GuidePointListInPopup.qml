import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../Custom"

Frame {
    id: leftPanelFrame

    padding: 0

    height: parent.height

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

}

//Page {
//    id: page
//    anchors.fill: parent

//    Menu {
//            id: pointMenu
//            padding: 0
//            width: parent.width
//            x: parent.width
//            visible: isOpen

//            background: Rectangle {
//                color: Style.lightGreyBackground
//                border.color: Style.lightGreyBorder
//                radius: 5
//            }

//            ColumnLayout {
//                anchors {
//                    left: parent.left
//                    right: parent.right
//                }

//                Repeater {
//                    model: points
//                    delegate: delegate
//                }

//                Component {
//                    id: delegate
//                    Rectangle {
//                        visible: isOpen
//                        height: isOpen ? 37 : 0
//                        anchors.left: parent.left
//                        anchors.right: parent.right
//                        color: "transparent"

//                        /// The blue rectangle on the selected item
//                        Rectangle {
//                            anchors.verticalCenter: parent.verticalCenter
//                            height: parent.height - 10
//                            anchors.left: parent.left
//                            anchors.right: parent.right
//                            color: (column.selectedGroup === groupName && column.selectedPoint === name) ? Style.selectedItemColor : "transparent"
//                        }

//                        MouseArea {
//                            onClicked: {
//    //                            column.selectedGroup = groupName;
//    //                            column.selectedPoint = name;
//                                console.log("groupName =  name = " + name)
//                                robotModel.savePlaceSignal(robotModel.ipRobot, name, posX, posY, orientation, home)
//                            }
//                            anchors.fill: parent
//                        }

//                        /// The item displaying the name of the point/group
//                        CustomLabel {
//                            text: {
//                                qsTr(name)
//                            }
//                            color: Style.blackMenuTextColor
//                            anchors.verticalCenter: parent.verticalCenter
//                            anchors.left: parent.left
//    //                        anchors.right: parent.right
//                            anchors.leftMargin: 5
//                            anchors.rightMargin: 5
//                        }


//                    }
//                }
//            }
//        }

//}

//Menu {
//    id: selectPointMenu
//    padding: 0
//    width: 140
//    property Points pointModel
//    property int currentMenuIndex: -1
//    property bool homeOnly: false
//    signal pointSelected(string name, double posX, double posY, int orientation)

//    background: Rectangle {
//        color: Style.lightGreyBackground
//        border.color: Style.lightGreyBorder
//        radius: 5
//    }

//    ColumnLayout {
//        spacing: 0
//        anchors {
//            left: parent.left
//            right: parent.right
//        }

//        Repeater {
//            model: pointModel
//            delegate: PopupMenuItem {
//                labelText: groupName
//                Layout.preferredHeight: Style.menuItemHeight+1
//                Layout.preferredWidth: parent.width
//                leftPadding: Style.menuItemLeftPadding

//                Image {
//                    id: arrow
//                    asynchronous: true
//                    source: "qrc:/icons/arrow"
//                    fillMode: Image.Pad // For not stretching image
//                    anchors.verticalCenter: parent.verticalCenter
//                    anchors.right: parent.right
//                    anchors.rightMargin: 12
//                }
//                onHoveredChanged: if(visible && !pointMenu.visible) currentMenuIndex = index /// desktop
//                onClicked: if(visible && !pointMenu.visible) currentMenuIndex = index /// android


//                Menu {
//                    id: pointMenu
//                    padding: 0
//                    width: 140z
//                    x: parent.width
//                    visible: currentMenuIndex === index && (homeOnly ? pointModel.getNbHome(groupName) > 0 : true)

//                    background: Rectangle {
//                        color: Style.lightGreyBackground
//                        border.color: Style.lightGreyBorder
//                        radius: 5
//                    }

//                    ColumnLayout {
//                        spacing: 0
//                        anchors {
//                            left: parent.left
//                            right: parent.right
//                        }

//                        Repeater {
//                            model: points
//                            delegate: PopupMenuItem {
//                                leftPadding: Style.menuItemLeftPadding
//                                Layout.preferredHeight: homeOnly ? (home ? Style.menuItemHeight+1 : 0) : Style.menuItemHeight+1
//                                Layout.preferredWidth: parent.width
//                                visible: homeOnly ? home : true
//                                labelText: name

//                                onTriggered: {
//                                    console.log("selectPointMenu.pointSelected");
//                                    selectPointMenu.pointSelected(name, posX, posY, orientation)
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }
//    }
//}
