import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Window 2.2
import "../../Helper/style.js" as Style
import "../../Model/Robot"
import "../../Model/Tutorial"
import "../../View/Custom/"
import "../../View/Tutorial"

Window {

    id: scanWindow
    objectName: "scanWindow"

    width: 1000
    minimumWidth: 800
    height: 700
    minimumHeight: 600

    property Robots robotModel
    property Tutorial tutorial
    property bool closeOnSave: false

    signal resetMapConfiguration(string file_name, bool scan)
    signal discardMap(bool discard)

    onVisibleChanged: {
        /// When we close the scan window, we want all the robot to stop scanning
        if(visible){
            scanMapLeftMenu.reset();
            scanMapLeftMenu.resetScanMaps();
            discardMap(false);
            console.log("size scanleftmenu " + scanMapLeftMenu.width + " " + scanMapLeftMenu.height + " " + height);
            closeOnSave = false;
        } else {
            scanMapLeftMenu.stopAllScans(!closeOnSave);
            scanMapLeftMenu.clear();
        }
    }

    Shortcut {
        sequence: "u"
        onActivated: scanMapLeftMenu.teleop(scanMapLeftMenu.selectedIp, 0)
    }

    Shortcut {
        sequence: "i"
        onActivated: scanMapLeftMenu.teleop(scanMapLeftMenu.selectedIp, 1)
    }

    Shortcut {
        sequence: "o"
        onActivated: scanMapLeftMenu.teleop(scanMapLeftMenu.selectedIp, 2)
    }

    Shortcut {
        sequence: "j"
        onActivated: scanMapLeftMenu.teleop(scanMapLeftMenu.selectedIp, 3)
    }

    Shortcut {
        sequence: "k"
        onActivated: scanMapLeftMenu.teleop(scanMapLeftMenu.selectedIp, 4)
    }

    Shortcut {
        sequence: "l"
        onActivated: scanMapLeftMenu.teleop(scanMapLeftMenu.selectedIp, 5)
    }

    Shortcut {
        sequence: "m"
        onActivated: scanMapLeftMenu.teleop(scanMapLeftMenu.selectedIp, 6)
    }

    Shortcut {
        sequence: ","
        onActivated: scanMapLeftMenu.teleop(scanMapLeftMenu.selectedIp, 7)
    }

    Shortcut {
        sequence: "."
        onActivated: scanMapLeftMenu.teleop(scanMapLeftMenu.selectedIp, 8)
    }

    ScanMapLeftMenu {
        id: scanMapLeftMenu
        scanWindowWidth: scanWindow.width
        scanWindowHeight: scanWindow.height
        visible: scanWindow.visible
        robotModel: scanWindow.robotModel
        tutorial: scanWindow.tutorial
        anchors {
            top: parent.top
            left: parent.left
            bottom: parent.bottom
        }
        onCancelScan: scanWindow.close();
        onSaveScan: closeOnSave = true
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

            function adjustSize(_width, _height){
                console.log("adjusting scan size to " + _width + " " + _height);
                width = _width;
                height = _height;
            }

            width: 2496
            height: 2496

            color: "#cdcdcd"

            MouseArea {
                anchors.fill: parent
                clip: true
                acceptedButtons: Qt.LeftButton
                drag.target: parent

                onClicked: console.log("scan map " + mouseX + " " + mouseY + " width " + width + " height " + height + " " + robotModel.count + " " + robotModel.get(0).posX + " " + robotModel.get(0).posY)

                onWheel: {
                    var newScale = scanMap.scale + scanMap.scale * wheel.angleDelta.y / 120 / 10;
                    if(newScale > 0.20 && newScale < Style.maxZoom)
                        scanMap.scale = newScale;
                }
            }
        }
    }

    CustomDialog {
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

    function checkScanWindow(){
        /// Stop the scan if a scanning robot reconnect after the window has been closed
        if(!visible)
            robotModel.stopAllScanOnConnection();
    }

    function grabScannedMap(file_name){
        console.log("scan: grabbed called " + file_name.substring(7) + ".pgm");

        if(file_name.toString().lastIndexOf(".pgm") === -1)
            file_name += ".pgm";

        scanMap.grabToImage(function(result) {
            result.saveToFile(file_name.substring(7));
            scanWindow.resetMapConfiguration(file_name, true);
            scanWindow.close();
        });
    }
}
