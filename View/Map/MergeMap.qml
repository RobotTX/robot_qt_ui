import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQuick.Window 2.2
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Model/Robot/"
import "../Robot/"
import "../Custom/"
import "../../Model/Tutorial"
import "../Tutorial"

Window {

    id: window
    title: "Merge maps"
    objectName: "mergeMapWindow"

    width: 1000
    minimumWidth: 800
    height: 700
    minimumHeight: 600


    property Robots robotModel
    property Tutorial tutorial
    // no need to do it twice, on the hideEvent or the showEvent, both would work, here we clean the map on the hideEvent
    onVisibleChanged: {
        leftMenu.clearList();
        if(!visible)
            tutorialD.close();
        else {
            resetWidget();
            if(tutorial.isDisplayed("merge_map"))
                tutorialD.open()
        }
    }

    signal resetWidget()
    signal resetMapConfiguration(string file_name, bool scan)


    // the menu of the left from where u can rotate maps
    MergeMapsLeftMenu {
        id: leftMenu
        anchors {
            top: parent.top
            bottom: parent.bottom
            left: parent.left
        }
        robotModel: window.robotModel
        onCloseWidget: window.hide()
        onResetWidget: window.resetWidget()
        onDisplayTutorial: tutorialD.open()
    }

    Rectangle {

        clip: true

        anchors {
            left: leftMenu.right
            right: parent.right
            top: parent.top
            bottom: parent.bottom
        }

        Rectangle {

            id: mergedMap
            clip: true
            objectName: "mergeMapsView"

            function adjustSize(_width, _height){
                console.log("adjusting merge map size to " + _width + " " + _height)
                width = _width
                height = _height
            }

            width: 2496
            height: 2496

            color: "#cdcdcd"

            MouseArea {
                anchors.fill: parent
                clip: true
                acceptedButtons: Qt.LeftButton
                drag.target: parent

                onClicked: console.log(mouseX + " " + mouseY + " width " + width + " height " + height)

                onWheel: {
                    var newScale = mergedMap.scale + mergedMap.scale * wheel.angleDelta.y / 120 / 10;
                    if(newScale > 0.20 && newScale < Style.maxZoom)
                        mergedMap.scale = newScale;
                }
            }
        }
    }

    function grabMergedMap(_fileName){

        if(_fileName.toString().lastIndexOf(".pgm") === -1)
            _fileName += ".pgm";

        mergedMap.grabToImage(function(result) {
            result.saveToFile(_fileName.substring(7));
            window.resetMapConfiguration(_fileName, false);
            window.close();
        });
    }

    function cancelImportMap(){
        console.log("Cancelling import map");
        leftMenu.removeLastRobot();
        mapImportErrorDialog.open();
    }

    CustomDialog {
        id: mapImportErrorDialog
        title: "Import impossible"
        x: window.width / 2 - width / 2
        y: window.height / 2 - height / 2
        message: "Sorry, you can only merge maps of the same size";
        acceptMessage: "ok"
        /*standardButtons: Dialog.Ok
        Label {
            text: "Sorry, you can only merge maps of the same size";
        }*/
    }

    TutorialDialog {
        id: tutorialD
        x: window.width / 2 - width / 2
        y: window.height / 2 - height / 2
        feature: "merge_map"
        tutorial: window.tutorial
        Component.onCompleted: tutoMessage = tutorial.getMessage("merge_map")
    }
}
