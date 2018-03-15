import QtQuick 2.0
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {
    id: btn
    property string txt
    property string imgSrc
    property bool clickedButton

    height: 100
    width: 100

    contentItem: Text {
        text: btn.text
        font: btn.font
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        anchors {
            horizontalCenter: rect.horizontalCenter
            verticalCenter: rect.verticalCenter
        }
    }

    MouseArea {
        id: mouseArea
        anchors.fill: rect
        hoverEnabled: true
        onClicked: mouseArea.enabled = true;
    }

    background: Rectangle {
        id: rect
        width: 100
        height: 100
        radius: width*0.5
        color: btn.pressed ? Style.lightGreyBorder : btn.hovered ? Style.selectedItemColor : btn.checked ? Style.selectedItemColor : Style.lightGreyBackgroundHover
    }


//    Image {
//        id: icon
//        source: imgSrc
//        fillMode: Image.Pad // to not stretch the image
//        anchors{
//            verticalCenter: parent.verticalCenter
//            left: parent.left
//            leftMargin: 20
//        }
//    }

//    CustomLabel {
//        text: qsTr(txt)
////        color: "#262626"
//        color: "red"
//        anchors{
//            verticalCenter: rect.verticalCenter
////            leftMargin: 15
//        }
//    }
}

