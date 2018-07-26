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

    signal groupSelected(string groupName)


    Rectangle {
        id: groupItem
        height: 70
        anchors {
            left: parent.left
            right: parent.right
            leftMargin: 10
        }

        color: Style.backgroundColorItemGuide

        Button {
            id: btnGroup
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
                        "#4e82c7"
                    } else if (index === 1) {
                        "#64aaea"
                    } else if (index === 2) {
                        "#5fa5bc"
                    } else if (index === 3) {
                        "#c12b35"
                    } else if (index === 4) {
                        "#c1285b"
                    } else if (index === 5) {
                        "#c3237e"
                    } else if (index === 6) {
                        "#ff6666"
                    } else if (index === 7) {
                        "#3b358e"
                    }
                }
            }

            Image {
                id: icon
                source: "qrc:/icons/click_128128" //imgSrc
                fillMode: Image.Pad // to not stretch the image
                anchors{
//                                    verticalCenter: parent.verticalCenter
                    left: parent.left
                    leftMargin: 50
                    bottom: parent.bottom
                    bottomMargin: 30
                }
            }

            CustomLabel {
                text: qsTr(groupName)
                color: "white"
                width: parent.width
//                wrapModeLabel: Label.NoWrap
                anchors{
                    verticalCenter: parent.verticalCenter
                    horizontalCenter: parent.horizontalCenter
                    top: parent.top
                    topMargin: 100
                }
                font.pointSize: 20
                font.bold: true
            }

            onClicked: {
                groupSelected(groupName)
            }
        }


    }
}
