import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

Button {
    text: "Cancel"
    height: 23

    background: Rectangle {
        radius: 3
        border.width: 1
        border.color: Style.lightGreyBorder
    }
}
