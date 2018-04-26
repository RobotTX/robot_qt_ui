import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Path"
import "../../Model/Robot"
import "../Custom"

Column {
    id: groupListItem

    property Paths pathModel
    property Column column
    property Robots robotModel
    property string langue
    property string groupSelected

    signal groupSelected(string groupName)

    Repeater {
        model: pathModel.pathsGuide.length
        delegate: delegate
    }

    Component {
        id: delegate

        Rectangle {
            color: "yellow"
            width: 10
            height: 10
        }
    }

//        Rectangle {
//            id: groupItem
//            visible: true
//            height: 70
//            anchors {
//                left: parent.left
//                right: parent.right
//                leftMargin: 10
//            }

//            color: Style.lightGreyBackground

//            /// The blue rectangle on the selected item
//            Rectangle {
//                anchors.verticalCenter: parent.verticalCenter
//                height: parent.height - 10
//                anchors.left: parent.left
//                anchors.leftMargin: 20
//                anchors.right: parent.right
//                anchors.rightMargin: 20
//            }

//            MouseArea {
//                onClicked: {
//                    column.selectedGroup = groupName;
//                    column.selectedPath = "";
//                }
//                anchors.fill: parent
//            }

//            Button {
//                id: btnPath
//                height: 310
//                width: 230
//                anchors {
//                    left: parent.left
//                    top: parent.top
//                    topMargin: 5
//                }

//                background: Rectangle {
//                    color: {
//                        if (index === 0) {
//                            "#859900"
//                        } else if (index === 1) {
//                            "#2aa198"
//                        } else if (index === 2) {
//                            "#268bd2"
//                        } else if (index === 3) {
//                            "#6c71c4"
//                        } else if (index === 4) {
//                            "#d33682"
//                        } else if (index === 5) {
//                            "#dc322f"
//                        } else if (index === 6) {
//                            "#cb4b16"
//                        } else if (index === 7) {
//                            "#b58900"
//                        }
//                    }
//                }

//                Image {
//                    id: icon
//                    source: "qrc:/icons/add" //imgSrc
//                    fillMode: Image.Pad // to not stretch the image
//                    anchors{
//                        verticalCenter: parent.verticalCenter
//                        left: parent.left
//                        leftMargin: 20
//                    }
//                }

//                CustomLabel {
//                    text: qsTr(groupName)
//                    color: "#262626"
//                    anchors{
//                        verticalCenter: parent.verticalCenter
//                        left: icon.right
//                        leftMargin: 15
//                    }
//                }
//                onClicked: groupSelected(groupName)
//            }

//        }
}

//    Repeater {
//        model: {
//            console.log("paths = " + paths)
//            paths
//        }
//        delegate: delegatePaths
//    }

//    Component {
//        id: delegatePaths
//        Column {
//            Frame {
//                visible: groupIsOpen
//                height: visible ? 37 : 0
//                width : groupListItem.width
//                padding: 0

//                /// The blue rectangle on the selected item
//                background: Rectangle {
//                    anchors.verticalCenter: parent.verticalCenter
//                    height: parent.height - 10
//                    anchors.left: parent.left
//                    anchors.right: parent.right
//                }

//                MouseArea {
//                    anchors.fill: parent
//                    onClicked: {
//                        column.selectedGroup = groupName;
//                        column.selectedPath = pathName;
//                    }
//                }

//                /// The item displaying the name of the path/group
//                CustomLabel {
//                    text: qsTr(pathName)
//                    color: Style.blackMenuTextColor
//                    anchors {
//                        verticalCenter: parent.verticalCenter
////                        left: leftButton2.right
////                        right: rightOpenPath.left
////                        leftMargin: 5
////                        rightMargin: 5
//                    }
//                }
//            }
//        }
//    }

