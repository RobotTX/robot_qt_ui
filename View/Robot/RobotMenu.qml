import QtQuick 2.7
import QtQuick.Controls 2.0

Page {
    id: page
    anchors.fill: parent

    Frame {
        id: titleFrame
        height: 43
        anchors {
            left: parent.left
            top: parent.top
            right: parent.right
        }

        background: Rectangle {
            color: "#FAFAFA"
            border.color: "#d8d8d8"
            border.width: 1
        }

        Label {
            text: qsTr("Robot title")
            anchors.centerIn: parent
        }
    }

    Frame {
        anchors {
            left: parent.left
            top: titleFrame.bottom
            right: parent.right
            bottom: parent.bottom
        }

        background: Rectangle {
            color: "#FAFAFA"
            border.color: "#d8d8d8"
            border.width: 1
        }

        Label {
            text: qsTr("Robot Page")
            anchors.centerIn: parent
        }
    }
}

