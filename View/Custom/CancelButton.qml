import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {

    id: btn
    property string langue
    property string txt: langue == "English" ? "Cancel" : "取消"
    height: 23


    CustomLabel {
        text: qsTr(txt)
        font.pointSize: Style.ubuntuSubHeadingSize
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        color: "White"
    }

    background: Rectangle {
        radius: 3
        border.width: 1
        color: btn.pressed ? Style.whiteButtonPressed : Style.darkSkyBlue
        border.color: Style.lightGreyBorder
    }
}
