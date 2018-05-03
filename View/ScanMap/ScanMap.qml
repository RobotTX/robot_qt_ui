import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Window 2.2
import QtQml.Models 2.2
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Model/Robot"
import "../../Model/Tutorial"
import "../../View/Custom/"
import "../../View/Tutorial"

Window {

    id: scanWindow
    objectName: "scanWindow"

    width: 1000
//    minimumWidth: 800
    height: 700
//    minimumHeight: 600

    property Robots robotModel
    property Tutorial tutorial
    property string langue
    property bool closeOnSave: false
    property ScanMapLeftMenu _scanMapLeftMenu: scanMapLeftMenu

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
        langue: scanWindow.langue
        anchors {
            top: parent.top
            left: parent.left
            bottom: parent.bottom
        }
        onCancelScan: scanWindow.close();
        onSaveScan: closeOnSave = true
    }

    Rectangle {

        id: scanFrame

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

            width: 2496
            height: 2496

            // when new metadata arrive the width and the height or adjusted
            function adjustSize(_width, _height){
                console.log("adjusting scan size to " + _width + " " + _height);
//                width = _width;
//                height = _height;
                /// Change the position of scanMap so that we see the received map in the middle of the screen
//                x = -_width/2 + scanFrame.width/2
//                y = -_height/2 + scanFrame.height/2
                x = -width/2 + scanFrame.width/2
                y = -height/2 + scanFrame.height/2
                console.log("x = " + x + " y = " + y)
            }

            color: "#cdcdcd"

            MouseArea {
                anchors.fill: parent
                clip: true
                acceptedButtons: Qt.LeftButton
                drag.target: parent

                onClicked: console.log("scan map " + scanMap.x + " "  + scanMap.y + " " + mouseX + " " + mouseY + " width " + width + " height " + height + " " + robotModel.count + " " + robotModel.get(0).posX + " " + robotModel.get(0).posY)

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
        parent: ApplicationWindow.overlay
        x: (parent.width - width) / 2
        y: (parent.height - height) / 2

        onAccepted: scanMapLeftMenu.startScanning(ip)
        onRejected: scanMapLeftMenu.setBusy(ip, false)
    }

    function openRestartScanMessageDialog(ip){
        var titleChange = "";
        var messageChange = "";
        var cancelChange = "";
        var okChange = "";
        if (langue == "English") {
            titleChange = "是否要重启扫描 ?";
            messageChange = "机器人已经重启，是否重启扫描（之前的扫描将被移除）?";
            cancelChange = "取消";
            okChange = "是";
        } else {
            titleChange = "Do you wish to restart the scan ?";
            messageChange = "The robot restarted, do you wish to restart the scan and erase the previous one ?";
            cancelChange = "Cancel";
            okChange = "Ok";
        }

        if(!dualChoiceMessageDialog.visible){
            dualChoiceMessageDialog.title =  qsTr(titleChange);
            dualChoiceMessageDialog.ip = ip;
            dualChoiceMessageDialog.message = qsTr(messageChange);
            dualChoiceMessageDialog.rejectMessage = qsTr(cancelChange);
            dualChoiceMessageDialog.acceptMessage = qsTr(okChange)
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

    function centerOnRobot(robot_x, robot_y, scan_map_item_x, scan_map_item_y){
        // position of the center of the frame in which we display the map in map coordinates
        var pos_finale = scanFrame.mapToItem(scanMap, scanFrame.width/2, scanFrame.height/2);
        scanMap.x += (pos_finale.x - (robot_x + scan_map_item_x)) * scanMap.scale;
        scanMap.y += (pos_finale.y - (robot_y + scan_map_item_y)) * scanMap.scale;
    }
}
