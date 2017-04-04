import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Model/Path"
import "../../Model/Robot"
import "../Path"

Frame {
    id: frame
    property Robots robotModel
    property Paths pathModel
    signal pathSelected(string _pathName, string _groupName)

    height: noPathItem.visible ? noPathItem.height : pathItem.height
    padding: 0

    background: Rectangle {
        color: "transparent"
    }

    Item {
        id: noPathItem
        visible: pathName === ""
        height: 23

        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
        }

        Button {
            id: assignPath
            text: "Assign Path"
            height: parent.height - 2
            padding: 0

            background: Rectangle {
                color: "white"
                border.width: 1
                border.color: Style.lightGreyBorder
                radius: 3
            }

            anchors {
                verticalCenter: parent.verticalCenter
                left: parent.left
                right: homeButton.left
                rightMargin: 12
            }

            onClicked: pathListInPopup.open()

            PathListInPopup {
                id: pathListInPopup
                x: assignPath.width
                onVisibleChanged: if(!visible) currentMenuIndex = -1
                pathModel: frame.pathModel
                onPathSelected: {
                    frame.pathSelected(pathName, groupName);
                    currentMenuIndex = -1;
                    close();
                }
            }
        }

        Button {
            id: homeButton

            height: parent.height - 2
            width: 32
            padding: 0

            background: Rectangle {
                color: "white"
                border.width: 1
                border.color: Style.lightGreyBorder
                radius: 3
            }

            anchors {
                verticalCenter: parent.verticalCenter
                right: parent.right
            }

            contentItem: Image {
                asynchronous: true
                source: homeName === "" ? "qrc:/icons/noHome" : "qrc:/icons/home"
                fillMode: Image.Pad // For not stretching image
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
            }
            onClicked: console.log(homeName === "" ? "I don't have a home" : "My home is " + homeName)
        }
    }

    Rectangle {
        id: pathItem
        visible: !noPathItem.visible
        height: pathIsOpen ? 145 : 59

        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
        }

        color: "white"
        border.width: 1
        border.color: Style.lightGreyBorder
        radius: 3


        Item {
            id: topItem
            height: 28

            anchors {
                top: parent.top
                left: parent.left
                right: parent.right
            }


            Button {
                id: hideShowPathButton

                height: parent.height - 2
                width: 32
                padding: 0

                background: Rectangle {
                    color: "white"
                }

                anchors {
                    verticalCenter: parent.verticalCenter
                    left: parent.left
                    leftMargin: 8
                }

                contentItem: Image {
                    asynchronous: true
                    source: pathIsVisible ? "qrc:/icons/visible" : "qrc:/icons/invisible"
                    fillMode: Image.Pad // For not stretching image
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                }
                onClicked: {
                    robotModel.hideShowPathOnMap(ip);
                    pathModel.visiblePathChanged();
                }
            }

            Label {
                text: qsTr(pathName)
                height: parent.height
                color: Style.blackMenuTextColor

                verticalAlignment: Text.AlignVCenter

                anchors {
                    verticalCenter: parent.verticalCenter
                    left: hideShowPathButton.right
                    right: expandPathButton.left
                    leftMargin: 5
                    rightMargin: 5
                }

                maximumLineCount: 1
                elide: Text.ElideRight
            }

            Button {
                id: expandPathButton

                height: parent.height - 2
                width: 32
                padding: 0

                background: Rectangle {
                    color: "white"
                }

                anchors {
                    verticalCenter: parent.verticalCenter
                    right: parent.right
                    rightMargin: 8
                }

                contentItem: Image {
                    asynchronous: true
                    source: pathIsOpen ? "qrc:/icons/fold" : "qrc:/icons/unfold"
                    fillMode: Image.Pad // For not stretching image
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                }
                onClicked: robotModel.openPath(ip)
            }

            Rectangle {
                id: borderBottom
                height: 1
                width: parent.width
                anchors.bottom: parent.bottom
                color: Style.lightGreyBorder
            }
        }



        Item {
            visible: pathIsOpen
            anchors {
                top: topItem.bottom
                bottom: bottomItem.top
                left: parent.left
                right: parent.right
            }

            Flickable {
                id: flick
                ScrollBar.vertical: ScrollBar { }
                contentHeight: contentItem.childrenRect.height
                anchors.fill: parent
                clip: true


                Rectangle {
                    anchors.fill: parent
                    color: "transparent" //Style.lightGreyBackground
                }

                Column {
                    /// The list containing both the graphical and model of the robots in the menu
                    Repeater {
                        model: pathPoints
                        delegate: delegatePathPoint
                    }

                    Component {
                        id: delegatePathPoint
                        Frame {
                            height: 25
                            width: flick.width
                            padding: 0
                            background: Rectangle {
                                anchors.fill: parent
                                color: "transparent"
                            }

                            Rectangle {
                                height: 15
                                width: 2
                                color: stage >= index ? Style.darkSkyBlue : "#d7d7d7"
                                anchors.horizontalCenter: rect.horizontalCenter
                                anchors.bottom: rect.top
                                visible: index > 0
                            }

                            Rectangle {
                                id: rect
                                height: 10
                                width: height
                                radius: height
                                color: "#d7d7d7"
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.left: parent.left
                                anchors.leftMargin: 25
                            }

                            /// The item displaying the name of the path/group
                            Label {
                                text: qsTr(pathPointName)
                                font.pixelSize: 14
                                color: Style.midGrey2
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.left: rect.right
                                anchors.right: parent.right
                                anchors.leftMargin: 10
                                anchors.rightMargin: 5
                                maximumLineCount: 1
                                elide: Text.ElideRight
                            }
                        }
                    }
                }
            }

            Rectangle {
                height: 1
                width: parent.width
                anchors.bottom: parent.bottom
                color: Style.lightGreyBorder
            }
        }

        Item {
            id: bottomItem
            height: 28

            anchors {
                bottom: parent.bottom
                left: parent.left
                right: parent.right
            }


            Button {
                id: playPausePathButton

                height: parent.height - 2
                width: 32
                padding: 0

                background: Rectangle {
                    color: "white"
                }

                anchors {
                    verticalCenter: parent.verticalCenter
                    left: parent.left
                    leftMargin: 8
                }

                contentItem: Image {
                    asynchronous: true
                    source: playingPath ? "qrc:/icons/pause" : "qrc:/icons/play"
                    fillMode: Image.Pad // For not stretching image
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                }
                onClicked: playingPath ? robotModel.pausePathSignal(ip) : robotModel.playPathSignal(ip)
            }

            Button {
                id: stopPathButton

                height: parent.height - 2
                width: 32
                padding: 0

                background: Rectangle {
                    color: "white"
                }

                anchors {
                    verticalCenter: parent.verticalCenter
                    horizontalCenter: parent.horizontalCenter
                }

                contentItem: Image {
                    asynchronous: true
                    source: "qrc:/icons/stop"
                    fillMode: Image.Pad // For not stretching image
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                }
                onClicked: robotModel.stopPathSignal(ip)
            }

            Button {
                id: goHomeButton

                height: parent.height - 2
                width: 32
                padding: 0

                background: Rectangle {
                    color: "white"
                }

                anchors {
                    verticalCenter: parent.verticalCenter
                    right: parent.right
                    rightMargin: 8
                }

                contentItem: Image {
                    asynchronous: true
                    source: homeName === "" ? "qrc:/icons/noHome" : "qrc:/icons/home"
                    fillMode: Image.Pad // For not stretching image
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                }
                onClicked: console.log(homeName === "" ? "I don't have a home" : "Let's go home to " + homeName)
            }
        }
    }
}
