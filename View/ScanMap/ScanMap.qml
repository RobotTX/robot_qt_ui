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
        if(!visible){
            scanMapLeftMenu.reset();
            scanMapLeftMenu.resetScanMaps();
        }
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

            function adjustSize(_width, _height){
                console.log("adjusting scan size to " + _width + " " + _height)
                width = _width
                height = _height
            }

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

        if(file_name.toString().lastIndexOf(".pgm") === -1){
            console.log("you");
            scanMap.grabToImage(function(result) {
                result.saveToFile(file_name.substring(7) + ".pgm");
                // important to call the hide function here as this call is asynchronous and if you call hide outside
                // you will most likely hide the window before you can grab it and will end up grabbing nothing
                scanWindow.close();
            });
        }

        else scanMap.grabToImage(function(result) {
                                          result.saveToFile(file_name.substring(7));
                                          scanWindow.close();
        });
    }
}
