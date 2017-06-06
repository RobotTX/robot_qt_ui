import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import QtQuick.Dialogs 1.2
import "../../Helper/style.js" as Style
import "../../Model/Robot"
import "../Custom"
import "../Robot/"

Frame {

    id: leftMenu
    objectName: "mergeMapLeftMenu"

    padding: 0
    width: 180

    background: Rectangle {
        anchors.fill: parent
        border.color: Style.lightGreyBorder
        color: Style.lightGreyBackground
    }

    property Robots robotModel

    signal rotate(int _angle, int _index)
    signal removeMap(int _index)
    signal closeWidget()
    signal resetWidget()

    signal importMap(string file)
    signal exportMap(string file)
    signal getMapFromRobot(string ip)
    signal displayTutorial()

    // to store the robots whose map has been imported for merging (subset of the robotModel and only the name and ip attributes are used
    ListModel {
        id: mapsList

        function addRobot(name, ip, map_received){
            append({
                "name": name,
                "ip": ip,
                "map_received": map_received
            });
        }

        function setMapReceived(ip){
            for(var i = 0; i < count; i++)
                if(get(i).ip === ip)
                    setProperty(i, "map_received", true);
        }

        function removeRobot(id){
            remove(id)
        }

        function contains(ip){
            for(var i = 0; i < count; i++)
                if(get(i).ip === ip)
                    return true;
            return false;
        }
    }

    // all the maps from the robots we chose, can rotate each of them individually

    Item {
        id: addMapMenu
        height: 125

        anchors {
            left: parent.left
            top: parent.top
            right: parent.right
        }

        CustomLabel {
            id: mapLabel
            anchors {
                top: parent.top
                topMargin: 10
                left: parent.left
                leftMargin: 20
                right: helpButton.left
            }
            text: qsTr("Import a map :")
            verticalAlignment: Text.AlignVCenter

        }

        HelpButton {
            id: helpButton

            anchors {
                right: parent.right
                rightMargin: 5
                verticalCenter: mapLabel.verticalCenter
            }

            onClicked: displayTutorial()
        }

        // to load a map from your the computer file system
        ScanMergeMenuButton {
            id: importButton
            txt: "From a File"
            src: "qrc:/icons/load_map"

            anchors {
                top: helpButton.bottom
                topMargin: 5
                left: parent.left
                right: parent.right
            }

            onClicked: {
                loadFileDialog.from_robot = false;
                loadFileDialog.open();
            }
        }

        ScanMergeMenuButton {
            id: fromRobotButton
            txt: "From a Robot"
            src: "qrc:/icons/small_robot"

            anchors {
                top: importButton.bottom
                topMargin: 5
                left: parent.left
                right: parent.right
            }

            // the list of robots from which we can choose a map
            RobotListInPopup {
                id: robotsList
                robotModel: leftMenu.robotModel
                robotMapsList: mapsList
                x: fromRobotButton.width

                onRobotSelected: {
                    console.log("adding robot " + name + " " + ip)
                    mapsList.addRobot(name, ip, false)
                    leftMenu.getMapFromRobot(ip)
                }
            }

            onClicked: {
                console.log("click import map from robot button")
                loadFileDialog.from_robot = true
                robotsList.open()
            }
        }
    }

    ToolSeparator {
        id: separator
        orientation: Qt.Horizontal
        anchors {
            top: addMapMenu.bottom
            left: parent.left
            right: parent.right
            topMargin: 5
            leftMargin: 15
            rightMargin: 15
        }
    }

    Flickable {
        anchors {
            top: separator.bottom
            bottom: saveButton.top
            left: parent.left
            topMargin: 15
            leftMargin: 14
            bottomMargin: 14
            rightMargin: 4
            right: parent.right
        }

        clip: true

        ScrollBar.vertical: ScrollBar {  }

        contentHeight: contentItem.childrenRect.height

        ColumnLayout {

            spacing: 45

            anchors {
                left: parent.left
                right: parent.right
                top: parent.top
            }

            Repeater {

                model: mapsList

                MergeMapMenuItem {
                    onRemoveMap: {
                        console.log("removing id" + index);
                        leftMenu.removeMap(index);
                        mapsList.removeRobot(index);
                    }
                    onRotate: leftMenu.rotate(angle, id)
                }
            }
        }
    }

    CancelButton {
        id: cancelButton
        anchors {
            bottom: parent.bottom
            bottomMargin: 17
            left: parent.left
            leftMargin: 18
            right: parent.right
            rightMargin: 18
        }
        width: 70
        // received in the parent
        onClicked: leftMenu.closeWidget()
    }

    SaveButton {
        id: saveButton
        // we don't want to allow the user to save if there is no map to save at all
        canSave: mapsList.count > 0
        tooltip: "You need at least 1 map to save"
        anchors {
            bottom: cancelButton.top
            bottomMargin: 11
            left: parent.left
            leftMargin: 18
            right: parent.right
            rightMargin: 18
        }
        width: cancelButton.width
        // we handle the signal in the parent
        onReleased: if(saveButton.canSave) saveFileDialog.open()
    }

    FileDialog {
        id: saveFileDialog
        // format of files is pgm
        nameFilters: "*.pgm"
        // won't let you choose a file name if selectExisting is true
        selectExisting: false
        title: "Please choose a location for your map"
        // to start directly with that folder selected
        /// TODO pk tu fais des trucs comme ca
        folder: "/home/joan/Gobot/build-Gobot-Desktop_Qt_5_8_0_GCC_64bit-Debug/mapConfigs/"

        onAccepted: leftMenu.exportMap(fileUrl.toString())
    }

    // the window that actually opens to choose the file
    FileDialog {
        id: loadFileDialog
        property bool from_robot
        // allow only pgm files to be selected
        nameFilters: "*.pgm"
        title: "Import a map"
        /// TODO pk tu fais des trucs comme ca
        folder: "/home/joan/Gobot/build-Gobot-Desktop_Qt_5_8_0_GCC_64bit-Debug/mapConfigs/"
        selectMultiple: false
        onRejected: console.log("Canceled the save of a map")
        onAccepted: {
            console.log("gonna send file " << fileUrl);
            leftMenu.importMap(fileUrl.toString().substring(7));
            mapsList.addRobot(fileUrl.toString().substring(7), "" + mapsList.count+1, !from_robot)
        }
    }

    function clearList(){
        mapsList.clear();
    }

    function removeLastRobot(){
        mapsList.removeRobot(mapsList.count-1);
    }

    function setMapReceived(ip){
        mapsList.setMapReceived(ip);
    }
}
