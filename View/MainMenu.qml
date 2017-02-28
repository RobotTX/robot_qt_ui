import QtQuick 2.7
import QtQuick.Controls 2.0
import "../Customs"

GroupBox {
    id: menuGroupBox
    width: 66
    padding: 0
    rightPadding: 0
    bottomPadding: 0
    leftPadding: 0
    topPadding: 0
    spacing: 0

    anchors {
        left: parent.left
        top: parent.top
        bottom: parent.bottom
    }

    background: Rectangle {
        color: "#cc26303a"
    }

    MenuButton {
        id: robotButton
        txt: "Robot"
        imgSrc: "qrc:/icons/robot"
        anchors {
            left: parent.left
            top: parent.top
        }
    }

    MenuButton {
        id: pathButton
        txt: "Path"
        imgSrc: "qrc:/icons/path"
        anchors.top: robotButton.bottom
    }

    MenuButton {
        id: pointButton
        txt: "Point"
        imgSrc: "qrc:/icons/point"
        anchors.top: pathButton.bottom
    }

    MenuButton {
        id: mapButton
        txt: "Map"
        imgSrc: "qrc:/icons/map"
        anchors.top: pointButton.bottom
    }

    MenuButton {
        id: settingsButton
        txt: "Settings"
        imgSrc: "qrc:/icons/settings"
        anchors.bottom: parent.bottom
    }
}
