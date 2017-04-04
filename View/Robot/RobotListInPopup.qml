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
        implicitWidth: parent.width
        implicitHeight: robotModel.count * Style.menuItemHeight
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

            PopupMenuItem {
                enabled: !robotMapsList.contains(ip)

                anchors {
                    left: parent.left
                    right: parent.right
                }
                leftPadding: Style.menuItemLeftPadding
                Layout.preferredHeight: Style.menuItemHeight
                Layout.preferredWidth: parent.width

                Label {
                    color: enabled ? "black" : Style.darkGrey
                    text: qsTr(name)
                    anchors {
                        left: parent.left
                        right: parent.right
                        leftMargin: 20
                        rightMargin: 5
                        verticalCenter: parent.verticalCenter
                    }
                    maximumLineCount: 1
                    elide: Text.ElideRight
                }

                onTriggered: {
                    console.log("Selected robot " + name)
                    robotMenu.robotSelected(name, ip)
                    close()
                }
            }
        }
    }
}
