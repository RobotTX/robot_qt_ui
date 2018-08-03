import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../Custom"

Dialog {
    id: dialog
    modal: true
    property string ip
    property string message
    property string rejectMessage
    property string acceptMessage
    property string yesMessage
    property string colorPlayPause
    property string imgPausePlay
    signal yes()

    background: Rectangle {
        color: Style.backgroundColorItemGuide
        border.width: 2
        border.color: Style.goldenColor
        radius: 5
    }

    header: Label {
        id: customHeader
        background: Rectangle {
            color: "transparent"
        }
        anchors {
            top: parent.top
            left: parent.left
            topMargin: 5
            leftMargin: 10
        }

        font.bold: true
        text: qsTr(title)
    }

    contentItem: Rectangle {
        anchors.fill: parent
        color: "transparent"

        MouseArea {
            width: dialog.parent.width
            height: dialog.parent.height
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
        }

        Label {
            anchors {
                top: parent.top
                left: parent.left
                right: parent.right
                bottom: layout.top
                topMargin: customHeader.height + 10
                leftMargin: 10
                rightMargin: 10
            }
            text: qsTr(message)
            wrapMode: Text.WordWrap
            font.pointSize: Style.ubuntuTitleSize
            color: Style.goldenColor
            font.bold: true
        }

        RowLayout {
            id: layout
            anchors {
                left: parent.left
                leftMargin: 20
                right: parent.right
                rightMargin: 40
                bottom: parent.bottom
                bottomMargin: 30
            }

            Button {
                id: btnStop

                visible: yesMessage !== ""

                Image {
                    id: iconPause
                    source: imgPausePlay //"qrc:/icons/pauseDialog300" //imgSrc
                    fillMode: Image.Pad // to not stretch the image
                    anchors{
    //                                    verticalCenter: parent.verticalCenter
                        left: parent.left
                        leftMargin: 50
                        bottom: parent.bottom
                        bottomMargin: 30
                    }
                }

                background: Rectangle {
                    implicitWidth: 300
                    implicitHeight: implicitWidth
                    color: "transparent"
                }

                onReleased: dialog.yes()
            }

            Button {
                id: btnPausePlay

                visible: acceptMessage !== ""

                background: Rectangle {
                    implicitWidth: 300
                    implicitHeight: implicitWidth
                    color: "transparent"
                }

                Image {
                    id: iconStop
                    source: "qrc:/icons/stopDialog300" //imgSrc
                    fillMode: Image.Pad // to not stretch the image
                    anchors{
    //                                    verticalCenter: parent.verticalCenter
                        left: parent.left
                        leftMargin: 50
                        bottom: parent.bottom
                        bottomMargin: 30
                    }
                }

                onReleased: dialog.accept()
            }
        }
    }
}
