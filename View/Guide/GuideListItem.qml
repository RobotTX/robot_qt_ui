import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../../Model/Robot"
import "../Custom"

Column {
    id: groupListItem

    property Points pointModel
    property Column column
    property Robots robotModel
    property string langue
    property string selectedGroup
    property string selectedPoint
    property bool openGroup
    property bool clickedButton

    signal renameGroup(string name)
    signal savePlaceSignal(string ip, string name, double x, double y, double orientation, bool home)
    signal stopPathSignal(string ip)

    Rectangle {
        id: groupItem
        visible: true//!(groupName === Helper.noGroup)
//        height: visible ? 37 :0
        height: 70
        anchors.left: parent.left
        anchors.right: parent.right
//        color: "transparent"
        color: Style.lightGreyBackground

        /// The blue rectangle on the selected item
        Rectangle {
            anchors.verticalCenter: parent.verticalCenter
            height: parent.height - 10
            anchors.left: parent.left
            anchors.leftMargin: 20
            anchors.right: parent.right
            anchors.rightMargin: 20
            color: (column.selectedGroup === groupName && column.selectedPoint === "") ? Style.selectedItemColor : "transparent"
        }

        MouseArea {
            onClicked: {
                column.selectedGroup = groupName;
                column.selectedPoint = "";
                selectedGroup = column.selectedGroup;
                selectedPoint = column.selectedPoint;
            }
            anchors.fill: parent
        }

        SquareButton {
            imgSrc: "qrc:/icons/add"
            text: qsTr(groupName)
            anchors {
                left: parent.left
                leftMargin: 20
                top: parent.top
                topMargin: 5
            }

            checkable: true
            checked: openGroup

            onClicked: {
                console.log("group " + groupName);
                pointModel.hideShowGroup(groupName);
                openGroup = pointModel.openGroup;
                console.log("button clicked !!!");
                clickedButton = true;
            }
        }

        Menu {
//        Frame {
            id: pointMenu
            padding: 0
            width: 273
            height: contentItem.childrenRect.height + 150
            x: Style.widthGuideMenu + 10
            y: 0

            visible: openGroup

            background: Rectangle {
                color: Style.lightGreyBackground
                border.color: "transparent"
                radius: 5
            }

            property string colorC: "transparent"

//            ColumnLayout {
            GridLayout {
                id: idColumn
                columns: 2
                columnSpacing: 80
                rowSpacing: 80

//                RoundButton {
                Button {
                    property bool checkedStop
                    id: stopButton
                    text: "Stop"

                    contentItem: Text {
                        text: stopButton.text
                        font: stopButton.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        anchors {
                            horizontalCenter: rect.horizontalCenter
                            verticalCenter: rect.verticalCenter
                        }
                    }

                    MouseArea {
                        id: mouseArea
                        anchors.fill: rect
                        hoverEnabled: true
                        onClicked: {
                            console.log("button stop clicked");
                            robotModel.stopPathSignal(robotModel.ipRobot);
                        }
                    }

                    background: Rectangle {
                        id: rect
                        width: 100
                        height: 100
                        radius: width*0.5
                        color: stopButton.pressed ? Style.lightGreyBorder : stopButton.hovered ? Style.selectedItemColor : stopButton.checked ? Style.selectedItemColor : Style.lightGreyBackgroundHover
                    }
                }

                Repeater {
                    model: points
                    delegate: delegate
                }

                Component {
                    id: delegate
                    Rectangle {
                        visible: isOpen
                        height: isOpen ? 37 : 0
//                        anchors.left: parent.left
//                        anchors.right: parent.right
                        color: "white"

                        /// The blue rectangle on the selected item
                        Rectangle {
                            anchors.verticalCenter: parent.verticalCenter
                            height: parent.height - 10
                            anchors.left: parent.left
                            anchors.right: parent.right
                            color: (selectedGroup === groupName && selectedPoint === name) ? Style.selectedItemColor : "transparent"
                        }

                        /// The item displaying the name of the point/group
//                        RoundButton {
                        Button {
                            id: namePointLabelButton
                            text: {
                                qsTr(name)
                            }

                            contentItem: Text {
                                text: namePointLabelButton.text
                                font: namePointLabelButton.font
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                                anchors {
                                    horizontalCenter: rect.horizontalCenter
                                    verticalCenter: rect.verticalCenter
                                }
                            }

                            MouseArea {
                                id: mouseArea
                                anchors.fill: rect
                                hoverEnabled: true
                                onClicked: {
                                    selectedGroup = groupName;
                                    selectedPoint = name;
                                    robotModel.savePlaceSignal(robotModel.ipRobot, name, posX, posY, orientation, home);
                                    pathDialog.open();
                                }
                            }

                            background: Rectangle {
                                id: rect
                                width: 100
                                height: 100
                                radius: width*0.5
                                color: namePointLabelButton.pressed ? Style.lightGreyBorder : namePointLabelButton.hovered ? Style.selectedItemColor : namePointLabelButton.checked ? Style.selectedItemColor : Style.lightGreyBackgroundHover
                            }

                            CustomDialog {
                                id: pathDialog
                                x: -500
                                y: 200
                                height: 130
                                title: langue == "English" ? "警告"  : "Robot in motion"
                                message: "Robot " + robotModel.getName(robotModel.ipRobot) + " is processing to point " + selectedPoint + ". Press the button stop if you want to cancel the action."
                                acceptMessage: langue == "English" ? "请选择机器人或输入WiFi名称" : "Stop"
                                onAccepted: {
                                    robotModel.stopPathSignal(robotModel.ipRobot);
                                }


                            }

//                            Dialog {
//                                id: stopDialog
//                                modal: true

//                                background: Rectangle {
//                                    color: "#f3f3f3"
//                                    border.width: 2
//                                    border.color: "#bcb5b9"
//                                    radius: 5
//                                }


//                            }
                        }
                    }
                }
            }
        }
    }
}
