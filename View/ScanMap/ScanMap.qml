import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Window 2.2
import "../../Helper/style.js" as Style
import "../../Model/Robot"
import "../../View/Custom/"

Window {
    id: scanWindow
    objectName: "scanWindow"

    width: 1000
    minimumWidth: 800
    height: 700
    minimumHeight: 600

    property Robots robotModel

    ScanMapLeftMenu {
        robotModel: scanWindow.robotModel
        anchors {
            top: parent.top
            left: parent.left
            bottom: parent.bottom
        }
    }
}
