import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style


CheckBox {

    id: control

    height: 16

    indicator: Rectangle {
        id: indicator
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

    contentItem: CustomLabel {
        text: control.text
        font.pointSize: Style.ubuntuSubHeadingSize
        color: Style.greyText
        verticalAlignment: Text.AlignVCenter
        anchors {
            left: indicator.right
            leftMargin: 10
            right: parent.right
            rightMargin: 10
        }
    }
}

