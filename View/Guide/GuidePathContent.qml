import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../Custom"
import "../../Model/Path"
import "../../Model/Robot"

Frame {
    id: guidePathFrame
    padding: 0

    property Paths pathModel
    property Robots robotModel
    property string langue
    property variant pathsGuide: []
    property variant toto: {"tata": "hello", "tata": "hululu"}
    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        implicitHeight: parent.width
        implicitWidth: parent.width
        border.width: 1
    }

    Flickable {
        id: flickPath
        ScrollBar.vertical: ScrollBar { }
        contentHeight: contentItem.childrenRect.height
        anchors.fill: parent
        anchors.topMargin: 10

        Grid {
            id: columnIdPath
            columns: 4
            spacing: 0

            /// The list containing both the graphical and model of the paths in the menu
            Repeater {
                model: {
                    console.log("pathsGudie = " + toto)
                    toto

                }
                delegate: del
            }
        }

        Component {
            id: del

            Rectangle {
                id: rect
                visible: true
                height: 70
//                anchors {
//                    left: parent.left
//                    right: parent.right
//                    leftMargin: 10
//                }

                color: Style.lightGreyBackground

                Button {
                    id: btnPaths
                    height: 310
                    width: 230
                    anchors {
                        left: parent.left
                        top: parent.top
                        topMargin: 5
                    }

                    background: Rectangle {
                        color: {
                            if (index === 0) {
                                "#859900"
                            } else if (index === 1) {
                                "#2aa198"
                            }
                        }
                    }

                    Image {
                        id: icon
                        source: "qrc:/icons/add" //imgSrc
                        fillMode: Image.Pad // to not stretch the image
                        anchors{
                            verticalCenter: parent.verticalCenter
                            left: parent.left
                            leftMargin: 20
                        }
                    }

                    CustomLabel {
                        text: {
//                            qsTr(pathsGuide.toString())
                            tata
                        }
                        color: "#262626"
                        anchors{
                            verticalCenter: parent.verticalCenter
                            left: icon.right
                            leftMargin: 15
                        }
                    }
                    onClicked: {

                    }
                }

            }
        }
    }
}
