import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.1
import QtQuick.Layouts 1.3

import "../../Helper/style.js" as Style
import "../Custom"

Dialog {
    id: dialog
    width: 1000
    height: 700
    visible: true

    padding: 1

    Rectangle {

        anchors.right: parent.right
        anchors.left: parent.left
        anchors.top: parent.top
        height: undoButton.height

        border.color: Style.lightGreyBorder

        ToolBar {
/*
            background: Rectangle {
                border.color: "transparent"
                anchors.fill: parent
            }
*/
            RowLayout {

                ToolButton {
                    id: undoButton
                    Image {
                        source: "qrc:/icons/undo"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                ToolButton {
                    Image {
                        source: "qrc:/icons/redo"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                ToolButton {
                    Image {
                        source: "qrc:/icons/reset"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                ToolButton {
                    id: selectButton
                    Image {
                        source: "qrc:/icons/hand"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                Rectangle {
                    id: verticalSpaceBar1
                    anchors.left: selectButton.right
                    anchors.leftMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                    color: Style.lightGreyBorder
                    height: selectButton.height - 20
                    width: 2
                }

                ToolButton {
                    Image {
                        source: "qrc:/icons/white"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                ToolButton {
                    Image {
                        source: "qrc:/icons/grey"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                ToolButton {
                    id: blackButton
                    Image {
                        source: "qrc:/icons/black"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                Rectangle {
                    id: verticalSpaceBar2
                    anchors.left: blackButton.right
                    anchors.leftMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                    color: Style.lightGreyBorder
                    height: selectButton.height - 20
                    width: 2
                }

                ToolButton {
                    Image {
                        source: "qrc:/icons/dot"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                ToolButton {
                    Image {
                        source: "qrc:/icons/line"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                ToolButton {
                    Image {
                        source: "qrc:/icons/outline"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                ToolButton {
                    id: solidButton
                    Image {
                        source: "qrc:/icons/solid"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                Rectangle {
                    id: verticalSpaceBar3
                    anchors.left: solidButton.right
                    anchors.leftMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                    color: Style.lightGreyBorder
                    height: selectButton.height - 20
                    width: 2
                }

                ToolButton {
                    Image {
                        source: "qrc:/icons/decrease"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                ToolButton {
                    Image {
                        source: "qrc:/icons/robot"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                ToolButton {
                    id: increaseButton
                    Image {
                        source: "qrc:/icons/add"
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                Rectangle {
                    id: verticalSpaceBar4
                    anchors.left: increaseButton.right
                    anchors.leftMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                    color: Style.lightGreyBorder
                    height: selectButton.height - 20
                    width: 2
                }
            }

            ToolButton {
                id: closeButton
                anchors.right: saveButton.left
                Image {
                    source: "qrc:/icons/closeBtn"
                    anchors.verticalCenter: closeButton.verticalCenter
                    anchors.horizontalCenter: closeButton.horizontalCenter
                }
            }

            ToolButton {
                id: saveButton
                // could not find better than repositionning the button manually (10 is the margin)
                x: dialog.width - 10 - undoButton.width
                Image {
                    source: "qrc:/icons/save"
                    anchors.verticalCenter: saveButton.verticalCenter
                    anchors.horizontalCenter: saveButton.horizontalCenter
                }
            }
        }
    }


}
