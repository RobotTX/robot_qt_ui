import QtQuick 2.7
import QtQuick.Controls 2.1
import QtQml.Models 2.2
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../../Model/Path"
import "../MainMenu"
import "../Point"

Frame {
    id: mapViewFrame
    objectName: "mapViewFrame"

    // declared as properties so that main.qml can send them to the C++ side, sending mapImage.scale instead of zoom does not work
    property double centerX: mapImage.x
    property double centerY: mapImage.y
    property double zoom: mapImage.scale

    // this is to be able to display messages at the top from other classes
    property TopView topView: topViewId

    property string mapSrc

    property Points pointModel
    property Paths pathModel
    property Paths tmpPathModel
    property bool useTmpPathModel

    Connections {
        target: pathModel
        onVisiblePathChanged: canvas.requestPaint()
    }

    Connections {
        target: tmpPathModel
        onVisiblePathChanged: canvas.requestPaint()
    }

    property PointView tmpPointView: PointView {
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
        id: topViewId
        onSavePosition: emitPosition()
        onLoadPosition: mapViewFrame.loadPosition()
        /// If we have a map, the mapImage is visible
        /// so we enable the buttons to save/load the state of the map
        hasMap: mapImage.visible
    }

    EmptyMap {
        id: emptyMap
        anchors.fill: parent
    }

    Frame {
        id: imageFrame
        anchors.fill: parent
        padding:0

        Image {
            id: mapImage
            asynchronous: true
            visible: false
            source: mapSrc
            fillMode: Image.PreserveAspectFit // For not stretching image

            smooth: false

            /// Canvas to display the paths dotted line on the map
            Canvas {
                id:canvas
                anchors.fill: parent
                smooth: false
                onPaint:{
                    var ctx = canvas.getContext('2d');
                    ctx.dashedLineTo = function (fromX, fromY, toX, toY, pattern) {
                        // Our growth rate for our line can be one of the following:
                        //   (+,+), (+,-), (-,+), (-,-)
                        // Because of this, our algorithm needs to understand if the x-coord and
                        // y-coord should be getting smaller or larger and properly cap the values
                        // based on (x,y).
                        var lt = function (a, b) { return a <= b; };
                        var gt = function (a, b) { return a >= b; };
                        var capmin = function (a, b) { return Math.min(a, b); };
                        var capmax = function (a, b) { return Math.max(a, b); };

                        var checkX = { thereYet: gt, cap: capmin };
                        var checkY = { thereYet: gt, cap: capmin };

                        if (fromY - toY > 0) {
                            checkY.thereYet = lt;
                            checkY.cap = capmax;
                        }
                        if (fromX - toX > 0) {
                            checkX.thereYet = lt;
                            checkX.cap = capmax;
                        }

                        this.moveTo(fromX, fromY);
                        var offsetX = fromX;
                        var offsetY = fromY;
                        var idx = 0, dash = true;
                        while (!(checkX.thereYet(offsetX, toX) && checkY.thereYet(offsetY, toY))) {
                            var ang = Math.atan2(toY - fromY, toX - fromX);
                            var len = pattern[idx];

                            offsetX = checkX.cap(toX, offsetX + (Math.cos(ang) * len));
                            offsetY = checkY.cap(toY, offsetY + (Math.sin(ang) * len));

                            if (dash) this.lineTo(offsetX, offsetY);
                            else this.moveTo(offsetX, offsetY);

                            idx = (idx + 1) % pattern.length;
                            dash = !dash;
                        }
                    };

                    ctx.clearRect(0, 0, canvas.width, canvas.height)
                    ctx.strokeStyle = "#929292";
                    ctx.lineWidth = 3;
                    ctx.beginPath();
                    if(useTmpPathModel){
                        for(var i = 0; i < tmpPathModel.count; i++)
                            for(var j = 0; j < tmpPathModel.get(i).paths.count; j++)
                                if(tmpPathModel.get(i).paths.get(j).pathIsVisible && tmpPathModel.get(i).paths.get(j).pathPoints.count > 1)
                                    for(var k = 1; k < tmpPathModel.get(i).paths.get(j).pathPoints.count; k++)
                                        ctx.dashedLineTo(tmpPathModel.get(i).paths.get(j).pathPoints.get(k-1).posX,
                                                         tmpPathModel.get(i).paths.get(j).pathPoints.get(k-1).posY,
                                                         tmpPathModel.get(i).paths.get(j).pathPoints.get(k).posX,
                                                         tmpPathModel.get(i).paths.get(j).pathPoints.get(k).posY,
                                                         [3, 5]);
                    } else {
                        for(var i = 0; i < pathModel.count; i++)
                            for(var j = 0; j < pathModel.get(i).paths.count; j++)
                                if(pathModel.get(i).paths.get(j).pathIsVisible && pathModel.get(i).paths.get(j).pathPoints.count > 1)
                                    for(var k = 1; k < pathModel.get(i).paths.get(j).pathPoints.count; k++)
                                        ctx.dashedLineTo(pathModel.get(i).paths.get(j).pathPoints.get(k-1).posX,
                                                         pathModel.get(i).paths.get(j).pathPoints.get(k-1).posY,
                                                         pathModel.get(i).paths.get(j).pathPoints.get(k).posX,
                                                         pathModel.get(i).paths.get(j).pathPoints.get(k).posY,
                                                         [3, 5]);
                    }
                    ctx.stroke();
                }
            }

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

            /// Repeater to display the points on the map
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


            /// Repeater to display the paths points on the map
            Repeater {
                model: useTmpPathModel ? tmpPathModel : pathModel
                delegate: Repeater {
                    model: paths
                    delegate: Repeater {
                        model: pathPoints
                        delegate: PointView {
                            _name: name
                            _isVisible: pathIsVisible
                            _groupName: pathName
                            type: index == 0 ? Helper.PointViewType.PATHPOINT_START : Helper.PointViewType.PATHPOINT
                            originX: posX - width/2
                            originY: posY - height
                            x: posX - width/2
                            y: posY - height
                        }
                    }
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
