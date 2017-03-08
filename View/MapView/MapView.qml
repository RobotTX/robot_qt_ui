import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQml.Models 2.2
import "../../Helper/style.js" as Style
import "../MainMenu"
import "../../Model/Point"

Frame {
    id: mapViewFrame
    objectName: "mapViewFrame"
    property string mapSrc
    property Points pointModel
    signal saveState(double posX, double posY, double zoom, string mapSrc)
    signal loadState()
    padding: 0

    TopView {
        id: topView
        onSaveState: mapViewFrame.saveState(mapImage.x, mapImage.y, mapImage.scale, mapSrc)
        onLoadState: mapViewFrame.loadState()
        /// If we have a map, the mapImage is visible
        /// so we enable the buttons to save/load the state of the map
        hasMap: mapImage.visible
    }

    EmptyMap {
        id: emptyMap
        objectName: "emptyMap"
        anchors.fill: parent
    }

    Image {
        id: mapImage
        asynchronous: true
        visible: false
        source: mapSrc
        fillMode: Image.PreserveAspectFit // For not stretching image

        smooth: false

        MouseArea {
            id: dragArea
            anchors.fill: parent

            drag.target: parent
            onWheel: {
                var newScale = mapImage.scale + mapImage.scale * wheel.angleDelta.y / 120 / 10;
                if(newScale > Style.minZoom && newScale < Style.maxZoom)
                    mapImage.scale = newScale;
            }
        }

        DelegateModel {
            /// TODO check this
            id: visualModel
            model: pointModel
            delegate: Image {
                visible: _groupName !== ""
                source: "qrc:/icons/pointView"
                x: _x
                y: _y
                asynchronous: true
                fillMode: Image.Pad
            }
        }

        ListView {
            anchors.fill: mapImage
            model: visualModel
            interactive: false
        }
    }

    function setMap(_mapSrc){
        console.log("setMap source : " + _mapSrc);
        mapSrc = "file:/" + _mapSrc;
        emptyMap.visible = false;
        mapImage.visible = true;
    }

    function setMapState(posX, posY, zoom){
        console.log("setMapState : " + posX + " | " + posY + " | " + zoom);
        if(zoom > Style.maxZoom)
            zoom = Style.maxZoom;
        else if(zoom < Style.minZoom)
            zoom = Style.minZoom;
        mapImage.scale = zoom;
        mapImage.x = posX;
        mapImage.y = posY;
    }
    /*
    function initialize(){
        for(var i = 0; i < pointModel.count; i++){
            if(pointModel.at(i)._groupName !== ""){
                console.log("Creating pointview : " + pointModel.at(i)._name
                            + " | " + pointModel.at(i)._x + " | " + pointModel.at(i)._y);
                createQmlObject("Image {
                    source: \"qrc:/icons/pointView\"
                    objectName: \"pointView_\" + " + pointModel.at(i)._name +
                    "x: " + pointModel.at(i)._x +
                    "y: " + pointModel.at(i)._y +
                    "asynchronous: true
                    fillMode: Image.Pad
                }", mapImage, "/home/m-a/Desktop/pointView_debug.txt");
            }
        }
    }*/
}
