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
        id: scanMapLeftMenu
        robotModel: scanWindow.robotModel
        anchors {
            top: parent.top
            left: parent.left
            bottom: parent.bottom
        }
    }

    DualChoiceMessageDialog {
        id: dualChoiceMessageDialog
        x: parent.width / 2 - width / 2
        y: parent.height / 2 - height / 2

        onAccepted: scanMapLeftMenu.startScanning(ip)
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
