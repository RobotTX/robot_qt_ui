import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0
import "Customs/."

ApplicationWindow {
    id: applicationWindow
    visible: true
    width: 1000
    height: 700
    title: qsTr("Gobot :)")

    ColumnLayout {
        id: rowLayout
        x: 0
        y: 0
        spacing: 0
        anchors.top: parent.top
        anchors.bottom: parent.bottom

        GroupBox {
            id: groupBox
            width: 66
            padding: 0
            rightPadding: 0
            bottomPadding: 0
            leftPadding: 0
            topPadding: 0
            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
            Layout.fillWidth: false
            Layout.fillHeight: true
            spacing: 0

            background: Rectangle {
                color: "#27313a"
                opacity: 0.8
            }

            MenuButton {
                id: robotButton
                txt: qsTr("Robot")
                imgSrc: "qrc:/icons/robot_checked.png"
                checked: true
                x:0
                y:0
            }

            MenuButton {
                id: pathButton
                txt: qsTr("Path")
                imgSrc: "qrc:/icons/path.png"
                anchors.top: robotButton.bottom
            }

            MenuButton {
                id: pointButton
                txt: qsTr("Point")
                imgSrc: "qrc:/icons/point.png"
                anchors.top: pathButton.bottom
            }

            MenuButton {
                id: mapButton
                txt: qsTr("Map")
                imgSrc: "qrc:/icons/map.png"
                anchors.top: pointButton.bottom
            }

            MenuButton {
                id: settingsButton
                txt: qsTr("Settings")
                imgSrc: "qrc:/icons/settings.png"
                anchors.bottom: parent.bottom
            }
        }
    }
}
