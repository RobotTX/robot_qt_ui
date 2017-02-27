import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0

ApplicationWindow {
    id: applicationWindow
    visible: true
    width: 640
    height: 480
    title: qsTr("Gobot :)")

    ColumnLayout {
        id: rowLayout
        x: 0
        y: 0
        width: 640
        height: 480
        spacing: 0

        GroupBox {
            id: groupBox
            width: 200
            padding: 0
            rightPadding: 0
            bottomPadding: 0
            leftPadding: 0
            topPadding: 0
            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
            Layout.fillWidth: false
            Layout.fillHeight: true
            spacing: 0

            Flow {
                id: columnLayout1
                width: 100
                Layout.fillWidth: true
                Layout.fillHeight: true
                Button {
                    id: button5
                    text: qsTr("Robot")
                    checked: true
                    autoExclusive: true
                    checkable: true
                    Layout.alignment: Qt.AlignLeft | Qt.AlignTop
                }

                Button {
                    id: button6
                    text: qsTr("Path")
                    transformOrigin: Item.Center
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignLeft | Qt.AlignTop
                    autoExclusive: true
                    checkable: true
                }

                Button {
                    id: button7
                    text: qsTr("Point")
                    Layout.alignment: Qt.AlignLeft | Qt.AlignTop
                    autoExclusive: true
                    checkable: true
                }

                Button {
                    id: button8
                    text: qsTr("Map")
                    Layout.alignment: Qt.AlignLeft | Qt.AlignTop
                    autoExclusive: true
                    checkable: true
                }
            }
        }

        Button {
            id: button9
            text: qsTr("Settings")
            Layout.alignment: Qt.AlignLeft | Qt.AlignBottom
            autoExclusive: true
            checkable: true
        }

    }
}
