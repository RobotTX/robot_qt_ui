import QtQuick 2.7
import QtQuick.Controls 2.1

import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style


CheckBox {
    id: control
    height: 16

    indicator: Rectangle {
        implicitWidth: 16
        implicitHeight: 16
        radius: 3
        border.color: control.checked ? Style.darkSkyBlueBorder : "#cdcdcd"
        border.width: 1
        color: control.checked ? Style.darkSkyBlue : control.hovered ? "#d6d6d6" : "white"
        Image {
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
            source: "qrc:/icons/checkmark"
        }
    }

    contentItem: Text {
        text: control.text
        font.pointSize: 10
        elide: Text.ElideRight
        color: Style.greyText
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        leftPadding: control.indicator.width
    }
}

