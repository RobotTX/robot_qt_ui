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


    onVisibleChanged: {
        if(!visible)
            scanMapLeftMenu.reset();
    }

    ScanMapLeftMenu {
        id: scanMapLeftMenu
        visible: scanWindow.visible
        robotModel: scanWindow.robotModel
        anchors {
            top: parent.top
            left: parent.left
            bottom: parent.bottom
        }
        onCancelScan: scanWindow.close()
    }

    Rectangle {

        clip: true

        anchors {
            top: parent.top
            left: scanMapLeftMenu.right
            right: parent.right
            bottom: parent.bottom
        }

        Rectangle {

            id: scanMap
            clip: true
            objectName: "scanMapView"

            width: 2048
            height: 2048

            color: "#cdcdcd"

            MouseArea {
                anchors.fill: parent
                clip: true
                acceptedButtons: Qt.LeftButton
                drag.target: parent

                onClicked: console.log(mouseX + " " + mouseY + " width " + width + " height " + height)

                onWheel: {
                    var newScale = scanMap.scale + scanMap.scale * wheel.angleDelta.y / 120 / 10;
                    if(newScale > 0.20 && newScale < Style.maxZoom)
                        scanMap.scale = newScale;
                }
            }
        }
    }

    DualChoiceMessageDialog {
        id: dualChoiceMessageDialog
        x: parent.width / 2 - width / 2
        y: parent.height / 2 - height / 2

        onAccepted: scanMapLeftMenu.startScanning(ip)
        onRejected: scanMapLeftMenu.setBusy(ip, false)
    }

    function openRestartScanMessageDialog(ip){
        if(!dualChoiceMessageDialog.visible){
            dualChoiceMessageDialog.title = qsTr("Do you wish to restart the scan ?");
            dualChoiceMessageDialog.ip = ip;
            dualChoiceMessageDialog.message = "The robot restarted, do you wish to restart the scan and erase the previous one ?";
            dualChoiceMessageDialog.rejectMessage = "Cancel";
            dualChoiceMessageDialog.acceptMessage = "Ok";
            dualChoiceMessageDialog.open();
        }
    }
}
