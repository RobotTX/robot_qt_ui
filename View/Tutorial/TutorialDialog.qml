import QtQuick 2.7
import QtQuick.Controls 2.1
import "../Custom"
import "../../Helper/style.js" as Style
import "../../Model/Tutorial"

Dialog {

    id: tutorialDialog

    property Tutorial tutorial
    property string tutoMessage
    property string feature
    property string langue

    // to reset the box that asks the user if he wants to hide the message
    onVisibleChanged: box.checked = true

    title: feature

    height: 400
    width: 400

    header: Label {
        id: customHeader
        font.pointSize: Style.ubuntuSubHeadingSize
        background: Rectangle {
            color: "transparent"
        }
        anchors {
            top: parent.top
            left: parent.left
            topMargin: 10
            leftMargin: 10
        }

        font.bold: true
        text: qsTr(title)
    }

    contentItem: Rectangle {

        anchors.fill: parent
        color: "transparent"

        Label {
            id: label
            anchors {
                top: parent.top
                left: parent.left
                right: parent.right
                bottom: rect.top
                topMargin: customHeader.height + 20
                leftMargin: 10
                rightMargin: 10
            }
            text: qsTr(tutoMessage)
            font.pointSize: Style.ubuntuTextSize
            color: Style.midGrey2
            wrapMode: Text.WordWrap
        }

        Rectangle {
            id: rect
            height: 85
            color: "transparent"
            anchors {
                bottom: parent.bottom
                left: parent.left
                right: parent.right
            }

            SquareCheckBox {
                id: box
                anchors {
                    top: button.top
                    left: parent.left
                    leftMargin: 10
                    right: button.left
                }

                text: langue == "English" ? "Do not show this message again" : "不在显示此条信息"
                font.pointSize: Style.ubuntuSubHeadingSize
                }


            Button {
                id: button
                background: Rectangle {
                    radius: 3
                    color: Style.darkSkyBlue
                    border.width: 1
                    border.color: Style.darkSkyBlueBorder
                }

                anchors {
                    right: parent.right
                    rightMargin: 20
                    verticalCenter: parent.verticalCenter
                }
                height: 23
                width: 60
                visible: false
                text: langue == "English" ? "sdasasasd" : "是"

                onClicked: {
                    box.checked ? tutorialDialog.tutorial.hideMessage(tutorialDialog.feature) : tutorialDialog.tutorial.showMessage(tutorialDialog.feature)
                    tutorialDialog.close()
                }
            }
            Button {
                id: nextButton
                background: Rectangle {
                    radius: 3
                    color: Style.darkSkyBlue
                    border.width: 1
                    border.color: Style.darkSkyBlueBorder
                }

                anchors {
                    right: parent.right
                    rightMargin: 20
                    verticalCenter: parent.verticalCenter
                }
                height: 23
                width: 60
                visible: false
                text: langue == "English" ? "OK" : "是"

                onClicked: {
                    box.checked ? tutorialDialog.tutorial.hideMessage(tutorialDialog.feature) : tutorialDialog.tutorial.showMessage(tutorialDialog.feature)
                    tutorialDialog.close()
                }
            }
            Button {
                id: previousButton
                background: Rectangle {
                    radius: 3
                    color: Style.darkSkyBlue
                    border.width: 1
                    border.color: Style.darkSkyBlueBorder

                }

                anchors {
                    right: parent.right
                    rightMargin: 20
                    verticalCenter: parent.verticalCenter
                }
                height: 23
                width: 60
                contentItem: Text {
                    text: langue == "English" ? "OK" : "是"
                    color:"white"
                    font.pointSize: Style.ubuntuSubHeadingSize
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }


                onClicked: {
                    box.checked ? tutorialDialog.tutorial.hideMessage(tutorialDialog.feature) : tutorialDialog.tutorial.showMessage(tutorialDialog.feature)
                    tutorialDialog.close()
                }
            }
        }

    }
}
