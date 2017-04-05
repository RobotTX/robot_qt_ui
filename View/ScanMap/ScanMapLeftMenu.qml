import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/style.js" as Style
import "../../Model/Robot"
import "../Custom"
import "../Robot"

Frame {
    id: scanLeftMenuFrame
    objectName: "scanLeftMenuFrame"

    property Robots robotModel
    signal startScanning(string ip)
    signal stopScanning(string ip)
    signal playPauseScanning(string ip, bool scanning, bool scanningOnConnection)

    width: Style.smallMenuWidth
    padding: 0

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    // to store the robots whose map has been imported for merging (subset of the robotModel and only the name and ip attributes are used
    ListModel {
        id: scanningRobotsList

        function addRobot(name, ip){
            append({
                "name": name,
                "ip": ip,
                "mapReceived": false,
                "busy": true,
                "scanning": false
            });
            startScanning(ip)
        }

        function removeRobot(ip){
            for(var i = 0; i < count; i++)
                if(get(i).ip === ip)
                    remove(i);
        }

        function contains(ip){
            for(var i = 0; i < count; i++)
                if(get(i).ip === ip)
                    return true;
            return false;
        }

        function setMapReceived(ip){
            for(var i = 0; i < count; i++)
                if(get(i).ip === ip)
                    setProperty(i, "mapReceived", true);
        }

        function setBusy(ip, busy){
            for(var i = 0; i < count; i++)
                if(get(i).ip === ip)
                    setProperty(i, "busy", busy);
        }

        function setScanning(ip, scanning){
            for(var i = 0; i < count; i++)
                if(get(i).ip === ip)
                    setProperty(i, "scanning", scanning);
        }
    }

    NormalButton {
        id: addScan
        txt: "Add a Scan"
        imgSrc: "qrc:/icons/add"
        anchors{
            top: parent.top
            topMargin: 5
        }
        onClicked: robotListInPopup.open()

        RobotListInPopup {
            id: robotListInPopup
            x: addScan.width
            robotModel: scanLeftMenuFrame.robotModel
            robotMapsList: scanningRobotsList
            onRobotSelected: scanningRobotsList.addRobot(name, ip)
        }
    }

    Rectangle {
        id: rect
        color: Style.lightGreyBorder
        width: parent.width
        height: 2

        anchors {
            top: addScan.bottom
            left: parent.left
            right: parent.right
            topMargin: 5
            leftMargin: 15
            rightMargin: 15
        }
    }


    Component {
        id: delegate
        ScanMapListItem {
            width: flick.width
            onStopScanning: {
                scanLeftMenuFrame.stopScanning(ip);
                scanningRobotsList.setBusy(ip, true);
            }
            onPlayPauseScanning: {
                scanLeftMenuFrame.playPauseScanning(ip, scanning, robotModel.getScanningOnConnection(ip));
                scanningRobotsList.setBusy(ip, true);
            }
        }
    }

    Flickable {
        id: flick
        clip: true
        ScrollBar.vertical: ScrollBar { }
        contentHeight: contentItem.childrenRect.height
        anchors {
            top: rect.bottom
            left: parent.left
            right: parent.right
            bottom: saveButton.top
            topMargin: 5
            bottomMargin: 15
        }

        Column {
            /// The list containing both the graphical and model of the robots in the menu
            Repeater {
                model: scanningRobotsList
                delegate: delegate
            }
        }
    }

    SaveButton {
        id: saveButton
        anchors {
            bottom: cancelButton.top
            bottomMargin: 10
            left: parent.left
            leftMargin: 15
            right: parent.right
            rightMargin: 15
        }
    }

    CancelButton {
        id: cancelButton
        anchors {
            bottom: parent.bottom
            bottomMargin: 15
            left: parent.left
            leftMargin: 15
            right: parent.right
            rightMargin: 15
        }
    }

    function startedScanning(ip){
        scanningRobotsList.setScanning(ip, true);
        scanningRobotsList.setBusy(ip, false);
    }

    function stoppedScanning(ip){
        scanningRobotsList.removeRobot(ip);
    }

    function pausedScanning(ip){
        scanningRobotsList.setScanning(ip, false);
        scanningRobotsList.setBusy(ip, false);
    }

    function receivedScanMap(ip){
        scanningRobotsList.setMapReceived(ip);
    }
}
