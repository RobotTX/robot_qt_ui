import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQml.Models 2.2
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../MainMenu"
import "../../Model/Point"
import "../Point"

Frame {
    id: mapViewFrame
    objectName: "mapViewFrame"

    // declared as properties so that main.qml can send them to the C++ side, sending mapImage.scale instead of zoom does not work
    property double centerX: mapImage.x
    property double centerY: mapImage.y
    property double zoom: mapImage.scale

    // this is to be able to display messages at the top from other classes
    property TopView _topView: topView

    property string mapSrc

    property Points pointModel

    property PointView tmpPointView: PointView {
        objectName: "tmpPointView"
        parent: mapImage
        type: Helper.PointViewType.TEMP
        _name: "tmpPointView"
        _groupName: "tmpGroup"
        _isVisible: false
        originX: mapImage.width/2
        originY: mapImage.height/2
        x: mapImage.width/2
        y: mapImage.height/2
        signal tmpPointViewPosChanged()

        MouseArea {
            anchors.fill: parent
            drag {
                target: parent
                minimumX: 0 - parent.width / 2
                minimumY: 0 - parent.height
                maximumX: mapImage.width - tmpPointView.width / 2
                maximumY: mapImage.height - tmpPointView.height
            }
            onClicked: console.log("This is the temporary point")
            onPositionChanged: {
                if(drag.active)
                    parent.tmpPointViewPosChanged()
            }
        }
    }

    signal savePosition(double posX, double posY, double zoom, string mapSrc)
    signal loadPosition()

    padding: 0

    TopView {
        id: topView
        onSavePosition: emitPosition()
        onLoadPosition: mapViewFrame.loadPosition()
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
            anchors.fill: parent

            drag.target: parent
            onWheel: {
                var newScale = mapImage.scale + mapImage.scale * wheel.angleDelta.y / 120 / 10;
                if(newScale > Style.minZoom && newScale < Style.maxZoom)
                    mapImage.scale = newScale;
            }
            onClicked: {
                if(tmpPointView.visible){
                    tmpPointView.x = mouseX - tmpPointView.width / 2;
                    tmpPointView.y = mouseY - tmpPointView.height;
                    tmpPointView.tmpPointViewPosChanged()
                }
            }
        }

        Repeater {
            model: pointModel
            delegate: Repeater {
                model: points
                delegate: PointView {
                    _name: name
                    _isVisible: isVisible
                    _groupName: groupName
                    originX: posX - width/2
                    originY: posY - height
                    x: posX - width/2
                    y: posY - height
                }
            }
        }
    }

    function setMap(_mapSrc){
        console.log("setMap source : " + _mapSrc);
        mapSrc = "file:/" + _mapSrc;
        emptyMap.visible = false;
        mapImage.visible = true;
    }

    function setMapPosition(posX, posY, zoom){
        console.log("setMapPosition : " + posX + " | " + posY + " | " + zoom);
        if(zoom > Style.maxZoom)
            zoom = Style.maxZoom;
        else if(zoom < Style.minZoom)
            zoom = Style.minZoom;
        mapImage.scale = zoom;
        mapImage.x = posX;
        mapImage.y = posY;
    }

    function emitPosition(){
        // qml got a path of this format : file://path_understood_by_Qt, so we get rid of the first 6 characters
        mapViewFrame.savePosition(mapImage.x, mapImage.y, mapImage.scale, mapSrc.substring(6))
    }
}
