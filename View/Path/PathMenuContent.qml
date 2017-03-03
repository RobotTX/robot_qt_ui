import QtQuick 2.7
import QtQuick.Controls 2.0
import "../../Helper/style.js" as Style
import "../Custom"

Frame {
    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }


    EmptyMenu {
        txt: "You have no path, click the '+' buton to create a path."
        imgSrc: "qrc:/icons/big_path"
    }
}
