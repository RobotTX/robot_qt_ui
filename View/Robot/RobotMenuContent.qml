import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../Custom"

Frame {
    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }


    EmptyMenu {
        txt: "No robot connected. Make sure the robot and computer are connected to the same WIFI network."
        imgSrc: "qrc:/icons/big_robot"
    }
}
