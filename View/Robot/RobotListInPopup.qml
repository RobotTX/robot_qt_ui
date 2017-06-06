import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Robot"
import "../Custom"

Menu {
    id: robotMenu
    padding: 0
    width: 140

    property Robots robotModel
    property ListModel robotMapsList

    signal robotSelected(string name, string ip)

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        radius: 5
    }


    ColumnLayout {
        anchors {
            left: parent.left
            right: parent.right
        }

        Repeater {
            model: robotModel
            delegate: PopupMenuItem {
                labelText: name
                enabled: !robotMapsList.contains(ip) && !processingCmd
                Layout.preferredHeight: Style.menuItemHeight
                Layout.preferredWidth: parent.width
                leftPadding: Style.menuItemLeftPadding

                anchors {
                    left: parent.left
                    right: parent.right
                }

                onTriggered: {
                    console.log("Selected robot " + name)
                    robotMenu.robotSelected(name, ip)
                    close()
                }

                CustomBusyIndicator {
                    visible: processingCmd
                    running: processingCmd
                    anchors.fill: parent
                }
            }
        }
    }
}
