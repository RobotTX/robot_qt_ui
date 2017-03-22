import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style


CheckBox {
    id: control
    height: 16

    indicator: Rectangle {
        implicitWidth: 16
        implicitHeight: 16
        radius: 8
        border.color: "#cdcdcd"
        border.width: 1
        Rectangle {
            visible: control.checked
            color: "#4e4e4e"
            border.color: "#3e3e3e"
            radius: 3
            anchors.margins: 5
            anchors.fill: parent
        }
    }

    contentItem: Text {
        text: control.text
        //font: control.font
        font.pointSize: 10
        color: Style.greyText
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        leftPadding: control.indicator.width
    }
}
