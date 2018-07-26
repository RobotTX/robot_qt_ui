import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Dialogs 1.2
import "../../Helper/style.js" as Style
import "../../Model/Robot"
import "../../Model/Tutorial"
import "../Custom"
import "../Robot"
import "../Tutorial"

Frame {
    id: scanLeftMenuFrame
    objectName: "scanLeftMenuFrame"

    property Robots robotModel
    property string langue
    signal startScanning(string ip)
    signal startAutomaticScan(string ip)
    signal playPauseScanning(string ip, bool scanning, bool scanningOnConnection)
    signal playPauseExploring(string ip, bool exploring)
    signal sendTeleop(string ip, int index)
    signal cancelScan()
    signal resetScanMaps()
    signal saveScan(string file_name)
    signal removeMap(string ip)
    signal requestCoordinatesRobotAndScanMapItem(string ip)

    property Tutorial tutorial

    property int scanWindowWidth
    property int scanWindowHeight
    property string selectedIp
    property ListModel scanningRobots: scanningRobotsList

    width: Style.smallMenuWidth
    padding: 0

    onVisibleChanged: {
        selectedIp = "";
        if(!visible)
            tutorialD.close();
        else
            // when the scan window is opened, if the scan map message should be displayed then the dialog window is opened
//            if(tutorial.isDisplayed("scan_map")) {
//                tutorialD.open();
//                console.log("scanmapleftmenu = " + tutorial.isDisplayed())
//            } else if (tutorial.isDisplayed("scan_map_chinese")) {
//                tutorialDChinese.open();
//                console.log("scanmapleftmenu = " + tutorial.isDisplayed())
//            }
            langue == "English" ? tutorialD.open() : tutorialDChinese.open()
    }

    background: Rectangle {
        color: Style.lightGreyBackground
        border.color: Style.lightGreyBorder
        border.width: 1
    }

    Connections {
        target: robotModel
        onRobotDc: {
            scanningRobotsList.setBusy(ip, false);
            scanningRobotsList.setConnected(ip, false);
        }
        onRobotConnection: {
            scanningRobotsList.setConnected(ip, true);
        }
        onSetBusy: scanningRobotsList.setBusy(ip, busy);
    }

    // to store the robots whose map has been imported for scanning (subset of the robotModel and only the name and ip attributes are used

    ListModel {
        id: scanningRobotsList

        function addRobot(name, ip, automatically){
            append({
                "name": name,
                "ip": ip,
                "mapReceived": false,
                "busy": true,
                "scanning": false,
                "connected": true,
                "checkedIndex": -1,
                "onAutomatic": automatically
            });
            if(!automatically)
                startScanning(ip)
            else
                startAutomaticScan(ip)
        }
        function removeRobot(ip){
            for(var i = 0; i < count; i++)
                if(get(i).ip === ip)
                    remove(i);
            if(ip === selectedIp)
                selectedIp = "";
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

        function setConnected(ip, connected){
            for(var i = 0; i < count; i++)
                if(get(i).ip === ip)
                    setProperty(i, "connected", connected);
        }

        function reset(){
            for(var i = 0; i < count; i++)
                robotModel.stopScanning(get(i).ip, true);
            clear();
            selectedIp = "";
        }

        function stopAllScans(killGobotMove){
            for(var i = 0; i < count; i++)
                robotModel.stopScanning(get(i).ip, killGobotMove);
        }

        function setCheckedIndex(ip, index){
            for(var i = 0; i < count; i++)
                if(get(i).ip === ip)
                    setProperty(i, "checkedIndex", index);
        }

        function getCheckedIndex(ip){
            for(var i = 0; i < count; i++)
                if(get(i).ip === ip)
                    return get(i).checkedIndex;
            return -2;
        }

        function isOnAutomaticMode(ip){
            for(var i = 0; i < count; i++)
                if(get(i).ip === ip)
                    return get(i).onAutomatic;
        }

        function setExploring(ip, exploring){
            for(var i = 0; i < count; i++)
                if(get(i).ip === ip)
                    setProperty(i, "onAutomatic", exploring);
        }
    }

    Button {
        id: addScan

        height: 40
        padding: 0

        anchors {
            top: parent.top
            topMargin: 5
//            topMargin: 50
            left: parent.left
            right: parent.right
        }

        background: Rectangle {
            color: addScan.hovered ? Style.selectedItemColor : "transparent"
        }

        Image {
            id: icon
            source: "qrc:/icons/add"
            fillMode: Image.Pad // to not stretch the image
            anchors {
                verticalCenter: parent.verticalCenter
                left: parent.left
                leftMargin: 15
            }
        }

        CustomLabel {
            text: langue == "English" ? qsTr("Select Robot") : qsTr("选择机器人")
            color: "#262626"
            anchors{
                verticalCenter: parent.verticalCenter
                left: icon.right
                leftMargin: 5
            }
            font.pointSize: 11
        }

        HelpButton {
            id: helpButton

            anchors {
                verticalCenter: parent.verticalCenter
                right: parent.right
                rightMargin: 15
            }

            onClicked: langue == "English" ? tutorialD.open() : tutorialDChinese.open()
//                tutorialD.open()
        }

        onClicked: robotListInPopup.open()

        RobotListInPopup {
            id: robotListInPopup
            x: addScan.width
            robotModel: scanLeftMenuFrame.robotModel
            robotMapsList: scanningRobotsList
            onRobotSelected: scanningRobotsList.addRobot(name, ip, false)
        }
    }

    ToolSeparator {
        id: separator
        orientation: Qt.Horizontal
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
            enabled: !busy
            x: 1
            width: flick.width - 2
            selected: scanLeftMenuFrame.selectedIp === ip
            langue: scanLeftMenuFrame.langue
            onStopScanning: {
                if(robotModel.isConnected(ip)){
                    robotModel.stopScanning(ip, true);
                } else {
                    console.log("The robot was disconnected so we just remove the map");
                    scanLeftMenuFrame.removeMap(ip);
                    stoppedScanning(ip);
                }
            }
            onPlayPauseScanning: {
                if(robotModel.isConnected(ip))
                    scanLeftMenuFrame.playPauseScanning(ip, scanning, robotModel.getScanningOnConnection(ip));
            }
            onPlayPauseExploring: {
                if(robotModel.isConnected(ip))
                    scanLeftMenuFrame.playPauseExploring(ip, exploring);
            }
            onSendTeleop: scanLeftMenuFrame.teleop(ip, index)
            onSelect: scanLeftMenuFrame.selectedIp === ip ? scanLeftMenuFrame.selectedIp = "" : scanLeftMenuFrame.selectedIp = ip
            onCenterOnRobot: scanLeftMenuFrame.requestCoordinatesRobotAndScanMapItem(ip);
        }
    }

    Flickable {
        id: flick
        clip: true
        ScrollBar.vertical: ScrollBar { }
        contentHeight: contentItem.childrenRect.height
        anchors {
            top: separator.bottom
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
        canSave: scanningRobotsList.count > 0
        tooltip: langue == "English" ? "You need at least 1 map to save" : "保存时需要至少一个地图"
        langue: scanLeftMenuFrame.langue
        anchors {
            bottom: cancelButton.top
            bottomMargin: 10
            left: parent.left
            leftMargin: 15
            right: parent.right
            rightMargin: 15
        }
        onReleased: if(saveButton.canSave) saveFileDialog.open()
    }

    CancelButton {
        id: cancelButton
        langue: scanLeftMenuFrame.langue
        anchors {
            bottom: parent.bottom
            bottomMargin: 15
            left: parent.left
            leftMargin: 15
            right: parent.right
            rightMargin: 15
        }
        onClicked: cancelScan()
    }

    FileDialog {
        id: saveFileDialog
        // format of files is pgm
        nameFilters: "*.pgm"
        // won't let you choose a file name if selectExisting is true
        selectExisting: false
        title: langue == "English" ? "Pelase choose a location for your map" : "请选择一个路径"

        onAccepted: {
            var fileStr = fileUrl.toString();
            console.log("fileStr in scanmapleftMenu = " + fileStr);
            if (fileStr.indexOf(" ") >= 0) {
                warningDialog.open()
            } else {
                /// file:// for linux, file: for windows
                if(fileStr.indexOf("file://") === 0) {
                    fileStr = fileStr.slice(7);
                } else if(fileStr.indexOf("file:") === 0) {
                    fileStr = fileStr.slice(5);
                }

                scanLeftMenuFrame.saveScan(fileStr)
            }
        }
    }

    CustomDialog {
        id: warningDialog
        parent: ApplicationWindow.overlay
        x: (scanLeftMenuFrame.width - width) / 2
        y: (scanLeftMenuFrame.height - height) / 2
        height: 60
        title: langue == "English" ? "Warning dialog" : "警告窗口"
        acceptMessage: langue == "English" ? "\nSpace are not allowed" : "\nS地图名称不能包含空格"
    }

    TutorialDialog {
        id: tutorialD
        x: scanWindowWidth / 2 - width / 2
        y: scanWindowHeight / 2 - height / 2
        feature: "SCAN MAP"
        tutorial: scanLeftMenuFrame.tutorial
        langue: scanLeftMenuFrame.langue
        Component.onCompleted: tutoMessage = tutorial.getMessage("SCAN MAP")
    }

    TutorialDialog {
        id: tutorialDChinese
        x: scanWindowWidth / 2 - width / 2
        y: scanWindowHeight / 2 - height / 2
        feature: "扫描地图"
        tutorial: scanLeftMenuFrame.tutorial
        langue: scanLeftMenuFrame.langue
        Component.onCompleted: tutoMessage = tutorial.getMessage("扫描地图")
    }

    function startedScanning(ip){
        scanningRobotsList.setScanning(ip, true);
    }

    function stoppedScanning(ip){
        scanningRobotsList.removeRobot(ip);
    }

    function pausedScanning(ip){
        scanningRobotsList.setScanning(ip, false);
    }

    function playedExploration(ip){
        scanningRobotsList.setExploring(ip, true);
    }

    function pausedExploration(ip){
        scanningRobotsList.setExploring(ip, false);
    }

    function receivedScanMap(ip){
        scanningRobotsList.setMapReceived(ip);
    }

    function reset(){
        scanningRobotsList.reset();
    }

    function clear(){
        scanningRobotsList.clear();
    }

    function checkScanWindow(ip, scanning){
        /// Stop the scan if a scanning robot reconnect after the window has been closed
        if(scanning && (!scanLeftMenuFrame.visible || !scanningRobotsList.contains(ip)))
            robotModel.stopScanning(ip, true);
    }

    function stopAllScans(killGobotMove){
        scanningRobotsList.stopAllScans(killGobotMove);
    }

    function setBusy(ip, busy){
        scanningRobotsList.setBusy(ip, busy);
    }

    function teleop(ip, index){
        var checkedIndex = scanningRobotsList.getCheckedIndex(ip);
        if(ip !== "" && checkedIndex !== -2){
            /// Useful to know which button to check
            var newCheckedIndex = (index === 4 || checkedIndex === index) ? -1 : index;
            scanningRobotsList.setCheckedIndex(ip, newCheckedIndex);
//            console.log("Send teleop " + (newCheckedIndex === -1 ? 4 : newCheckedIndex));
            /// Send the command to the robot
            scanLeftMenuFrame.sendTeleop(ip, newCheckedIndex === -1 ? 4 : newCheckedIndex);
        } else
            console.log(ip === "" ? "No robot selected" : "Checked index could not be found : " + checkedIndex);
    }
}
