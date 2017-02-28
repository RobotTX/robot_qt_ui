import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0
import "View"

ApplicationWindow {
    id: applicationWindow
    visible: true
    width: 1000
    height: 700
    title: qsTr("Gobot :)")

    ColumnLayout {
        id: mainLayout
        spacing: 0

        anchors {
            left: parent.left
            top: parent.top
            right: parent.right
            bottom: parent.bottom
        }

        MainMenu {}
    }
}
