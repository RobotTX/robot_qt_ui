import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

RadioButton {

    property string txt

    Text {
        id: name
        text: qsTr(txt)
        color: Style.mapChoiceLabel
        anchors.leftMargin: 30
        anchors.verticalCenter: parent.verticalCenter
    }

    indicator: Rectangle {

        id: indicator
        anchors.verticalCenter: parent.verticalCenter
        anchors.right: name.left
        anchors.rightMargin: 6
        implicitWidth: 16
        implicitHeight: 16
        radius: 9
        border.width: 1

        Rectangle {

            anchors.fill: parent
            color: checked ? "#555" : "transparent"
            radius: 9
            anchors.margins: 4
        }
    }
}
