import QtQuick 2.0
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style

MenuItem {
    background: Rectangle {
        color: parent.hovered ? Style.lightGreyBackgroundHover : "transparent"
        radius: 5
    }
}
