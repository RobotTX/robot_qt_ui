import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {

    id: btn
    property string langue
    property string txt: langue == "English" ? "取消" : "Cancel"
    height: 23

    CustomLabel {
        text: qsTr(txt)
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        color: Style.backgroundColorDarkGuide
    }

    background: Rectangle {
        radius: 3
        border.width: 1
//        color: btn.pressed ? Style.whiteButtonPressed : "white"
        color: Style.goldenColor
//        border.color: Style.lightGreyBorder
        border.color: Style.goldenColorNormal
    }
}
