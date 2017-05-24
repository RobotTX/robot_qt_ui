import QtQuick 2.7
import QtQuick.Controls 1.4
import QtQuick.Controls 2.1
import QtQml.Models 2.2
import QtQuick.Layouts 1.3
import "../../Helper/style.js" as Style
import "../../Helper/helper.js" as Helper
import "../../Model/Point"
import "../../Model/Path"
import "../../Model/Robot"
import "../MainMenu"
import "../Point"
import "../Robot"
import "../Custom"
import "../"

Frame {
    id: mapViewFrame
    objectName: "mapViewFrame"

    // declared as properties so that main.qml can send them to the C++ side, sending zoomScale.xScale instead of zoom does not work
    property double centerX: mapImage.x
    property double centerY: mapImage.y
    property double zoom: zoomScale.xScale

    property string mapSrc

    property Points pointModel
    property Paths pathModel
    property Paths tmpPathModel
    property Robots robotModel

    property bool useTmpPathModel
    property bool useRobotPathModel

    signal posClicked(double x, double y)
    signal savePosition(double posX, double posY, double zoom, int mapRotations, string mapSrc)
    signal loadPosition()
    signal doubleClickedOnMap(double mouseX, double mouseY)

    Connections {
        target: pathModel
        onVisiblePathChanged: canvas.requestPaint()
    }

    Connections {
        target: tmpPathModel
        onVisiblePathChanged: canvas.requestPaint()
    }

    Connections {
        target: robotModel
        onVisiblePathChanged: canvas.requestPaint()
    }

    property PointView tmpPointView: PointView {
        parent: mapImage
        type: Helper.PointViewType.TEMP
        _name: "tmpPointView"
        _groupName: "tmpGroup"
        _isVisible: false
        pointPosX: 0
        pointPosY: 0
        tooltipText: "Drag me or click the map to modify my position"
        signal tmpPointViewPosChanged()

        mapOrientation: -topViewId.mapRotation

        MouseArea {
            anchors.fill: parent
            drag {
                target: parent
                minimumX: 0
                minimumY: 0
                maximumX: mapImage.width
                maximumY: mapImage.height
            }
            onClicked: console.log("This is the temporary point")
            onPositionChanged: {
                if(drag.active)
                    parent.tmpPointViewPosChanged()
            }
        }
    }

    padding: 0

    SplitView {
        anchors.fill: parent
        orientation: Qt.Vertical

        handleDelegate: Rectangle {}

        TopView {
            id: topViewId
            objectName: "topView"
            // qml got a path of this format : file://path_understood_by_Qt, so we get rid of the first 6 characters
            onSavePosition: mapViewFrame.savePosition(mapImage.x, mapImage.y, zoomScale.xScale, mapRotation, mapSrc.substring(6))
            onLoadPosition: mapViewFrame.loadPosition()
            /// If we have a map, the mapImage is visible
            /// so we enable the buttons to save/load the state of the map
            hasMap: mapImage.visible
            robotModel: mapViewFrame.robotModel
        }

        Item {

            Shortcut {
                sequence: "s"
                onActivated: mapImage.y = mapImage.y + zoomScale.xScale
            }
            Shortcut {
                sequence: "w"
                onActivated: mapImage.y = mapImage.y - zoomScale.xScale
            }
            Shortcut {
                sequence: "a"
                onActivated: mapImage.x = mapImage.x + zoomScale.xScale
            }
            Shortcut {
                sequence: "d"
                onActivated: mapImage.x = mapImage.x - zoomScale.xScale
            }

            clip: true
            EmptyMap {
                id: emptyMap
                anchors.fill: parent
            }

            MouseArea {
                id: mouseArea
                anchors.fill: parent

                onWheel: {

                    var oldPos = mapToItem(mapImage, width / 2, height / 2);
                    var factor = 1 + wheel.angleDelta.y / 120 / 10;
                    var newScale = zoomScale.xScale * factor;

                    /// Zoom into the image
                    if(newScale > Style.minZoom && newScale < Style.maxZoom)
                        zoomScale.xScale = newScale;

                    var newPos = mapToItem(mapImage, width / 2, height / 2);

                    /// Calculate the misplacement of the image so that we zoom in the middle of what we see and not in the middle of the map
                    var diff = Qt.point(newPos.x - oldPos.x, newPos.y - oldPos.y);
                    var res = Qt.point(diff.x* Math.cos(topViewId.mapRotation * (Math.PI / 180)) - diff.y * Math.sin(topViewId.mapRotation * (Math.PI / 180)),
                                     diff.x* Math.sin(topViewId.mapRotation * (Math.PI / 180)) + diff.y * Math.cos(topViewId.mapRotation * (Math.PI / 180)));

                    mapImage.x = mapImage.x + res.x * zoomScale.xScale;
                    mapImage.y = mapImage.y + res.y * zoomScale.xScale;
                }
            }

            Image {
                id: mapImage
                objectName: "mapImage"
                clip: true
                visible: false
                onSourceChanged: console.log("new source " + mapSrc)
                source: mapSrc
                fillMode: Image.PreserveAspectFit // For not stretching image
                // because the map could change (through the edit map function) it is not useful to cache it
                cache: false
                smooth: false
                rotation: topViewId.mapRotation

                transform: Scale {
                    id: zoomScale
                    yScale: xScale
                    origin.x: mapImage.width / 2
                    origin.y: mapImage.height / 2
                }

                /// Canvas to display the paths dotted line on the map
                Canvas {
                    id:canvas
                    anchors.fill: parent
                    smooth: false
                    onPaint:{
                        var ctx = canvas.getContext('2d');

                        ctx.clearRect(0, 0, canvas.width, canvas.height)
                        ctx.strokeStyle = "#929292";
                        ctx.lineWidth = 2;
                        ctx.beginPath();
                        var _spacing = 5;
                        if(useTmpPathModel){
                            for(var i = 0; i < tmpPathModel.count; i++)
                                for(var j = 0; j < tmpPathModel.get(i).paths.count; j++)
                                    if(tmpPathModel.get(i).paths.get(j).pathIsVisible && tmpPathModel.get(i).paths.get(j).pathPoints.count > 1)
                                        for(var k = 1; k < tmpPathModel.get(i).paths.get(j).pathPoints.count; k++)
                                            Helper.dashLine(ctx, tmpPathModel.get(i).paths.get(j).pathPoints.get(k-1).posX,
                                                             tmpPathModel.get(i).paths.get(j).pathPoints.get(k-1).posY,
                                                             tmpPathModel.get(i).paths.get(j).pathPoints.get(k).posX,
                                                             tmpPathModel.get(i).paths.get(j).pathPoints.get(k).posY,
                                                             [ctx.lineWidth, _spacing]);
                        } else {
                            if(useRobotPathModel){
                                for(var i = 0; i < robotModel.count; i++)
                                    if(robotModel.get(i).pathIsVisible && robotModel.get(i).pathPoints.count > 1)
                                        for(var j = 1; j < robotModel.get(i).pathPoints.count; j++){
                                            Helper.dashLine(ctx, robotModel.get(i).pathPoints.get(j-1).pathPointPosX,
                                                             robotModel.get(i).pathPoints.get(j-1).pathPointPosY,
                                                             robotModel.get(i).pathPoints.get(j).pathPointPosX,
                                                             robotModel.get(i).pathPoints.get(j).pathPointPosY,
                                                             [ctx.lineWidth, _spacing]);
                                        }
                            } else {
                                for(var i = 0; i < pathModel.count; i++)
                                    for(var j = 0; j < pathModel.get(i).paths.count; j++)
                                        if(pathModel.get(i).paths.get(j).pathIsVisible && pathModel.get(i).paths.get(j).pathPoints.count > 1)
                                            for(var k = 1; k < pathModel.get(i).paths.get(j).pathPoints.count; k++)
                                                Helper.dashLine(ctx, pathModel.get(i).paths.get(j).pathPoints.get(k-1).posX,
                                                                 pathModel.get(i).paths.get(j).pathPoints.get(k-1).posY,
                                                                 pathModel.get(i).paths.get(j).pathPoints.get(k).posX,
                                                                 pathModel.get(i).paths.get(j).pathPoints.get(k).posY,
                                                                 [ctx.lineWidth, _spacing]);
                            }
                        }
                        ctx.stroke();
                    }
                }

                MouseArea {
                    anchors.fill: parent
                    acceptedButtons: Qt.LeftButton | Qt.RightButton
                    hoverEnabled: true

                    drag.target: parent

                    onDoubleClicked: doubleClickedOnMap(mouseX, mouseY)

                    onClicked: {
                        if (mouse.button === Qt.LeftButton) {
                            if(tmpPointView.visible){
                                tmpPointView.setPos(mouseX,  mouseY);
                                tmpPointView.tmpPointViewPosChanged()
                            }
                            if(robotModel.count > 0)
                                console.log("robot pos " + robotModel.get(0).posX + " " + robotModel.get(0).posY)
                            if(useTmpPathModel){
                                tmpPathModel.addPathPoint(Math.round(mouseX) + ' ' + Math.round(mouseY),  "tmpPath", "tmpGroup", mouseX, mouseY, 0);
                                tmpPathModel.checkTmpPosition(tmpPathModel.get(0).paths.get(0).pathPoints.count - 1, mouseX, mouseY);
                                canvas.requestPaint();
                            }
                        } else if (mouse.button === Qt.RightButton){
                            posClicked(Math.round(mouseX), Math.round(mouseY));
                        }
                    }
                }

                /// Repeater to display the points on the map
                Repeater {
                    model: pointModel
                    delegate: Repeater {
                        model: points
                        delegate: PointView {
                            id: pointView
                            _name: name
                            _isVisible: useTmpPathModel ? true : isVisible
                            _groupName: groupName
                            originX: posX
                            originY: posY
                            x: posX - width / 2
                            y: posY - height
                            tooltipText: name
                            type: home ? Helper.PointViewType.HOME : Helper.PointViewType.PERM
                            mapOrientation: -topViewId.mapRotation
                            pointOrientation: orientation

                            MouseArea {
                                anchors.fill: parent
                                onClicked: {
                                    console.log("Clicked on " + _name + " in group " + _groupName + " " + _isVisible + " " + type)
                                    if(useTmpPathModel){
                                        tmpPathModel.addPathPoint(_name,  "tmpPath", "tmpGroup", posX, posY, 0);
                                        tmpPathModel.checkTmpPosition(tmpPathModel.get(0).paths.get(0).pathPoints.count - 1, posX, posY);
                                        canvas.requestPaint();
                                    }
                                }
                            }
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
                                id: pathPointView
                                _name: name
                                _isVisible: useRobotPathModel ? false : pathIsVisible
                                _groupName: pathName
                                type: index == 0 ? Helper.PointViewType.PATHPOINT_START : Helper.PointViewType.PATHPOINT
                                originX: posX
                                originY: posY
                                x: posX - width / 2
                                y: posY - height
                                tooltipText: name
                                mapOrientation: -topViewId.mapRotation

                                MouseArea {
                                    anchors.fill: parent
                                    onClicked: {
                                        console.log("Clicked on " + _name + " in group " + _groupName + " " + _isVisible + " " + type)
                                        if(useTmpPathModel){
                                            tmpPathModel.addPathPoint(_name,  "tmpPath", "tmpGroup", posX, posY, 0);
                                            tmpPathModel.checkTmpPosition(tmpPathModel.get(0).paths.get(0).pathPoints.count - 1, posX, posY);
                                            canvas.requestPaint();
                                        }
                                    }
                                    /// We want to be able to move the path points on the map when creating a path
                                    drag {
                                        target: useTmpPathModel ? parent : undefined
                                        minimumX: 0
                                        minimumY: 0
                                        maximumX: mapImage.width
                                        maximumY: mapImage.height
                                    }
                                    onPositionChanged: {
                                        if(drag.active){
                                            /// Need to do all of this to break the binding on the position of the image
                                            /// as when the pointView is moved, we want to set this value in the model
                                            /// which will update the pointView again...
                                            var _posX = pathPointView.x;
                                            var _posY = pathPointView.y;

                                            if(Math.round(posX) + ' ' + Math.round(posY) === name)
                                                name = Math.round(_posX) + ' ' + Math.round(_posY)

                                            var _oriX = pathPointView.pointPosX;
                                            var _oriY = pathPointView.pointPosY;
                                            pathPointView.pointPosX = _oriX;
                                            pathPointView.pointPosY = _oriY;
                                            pathPointView.x = _posX - pathPointView.width/2;
                                            pathPointView.y = _posY - pathPointView.height;

                                            posX = _posX;
                                            posY = _posY;
                                            tmpPathModel.checkTmpPosition(index, posX, posY);
                                            canvas.requestPaint();
                                        }
                                    }
                                }
                            }
                        }
                    }
                }


                /// Repeater to display the robots on the map
                Repeater {
                    model: robotModel
                    delegate: Item{
                        /// The robot on the map
                        RobotView {
                            id: robot
                            _name: name
                            _ip: ip
                            x: posX - width / 2
                            y: posY - height / 2
                            mapOrientation: -topViewId.mapRotation
                        }

                        /// The robot's home on the map
                        PointView {
                            id: homeView
                            _name: "Charging station of " + name
                            _isVisible: useRobotPathModel && homeX > -100
                            type: Helper.PointViewType.HOME
                            originX: homeX
                            originY: homeY
                            x: homeX - width / 2
                            y: homeY - height
                            tooltipText: "Charging station of " + name
                            mapOrientation: -topViewId.mapRotation
                            pointOrientation: homeOri
                        }

                        /// The robot's path on the map
                        Repeater {
                            model: pathPoints
                            delegate: PointView {
                                id: robotPathPointView
                                _name: pathPointName
                                _isVisible: useRobotPathModel ? pathIsVisible : false
                                _groupName: pathName
                                type: index == 0 ? Helper.PointViewType.PATHPOINT_START : Helper.PointViewType.PATHPOINT
                                originX: pathPointPosX
                                originY: pathPointPosY
                                x: pathPointPosX - width / 2
                                y: pathPointPosY - height
                                tooltipText: pathPointName
                                mapOrientation: -topViewId.mapRotation
                            }
                        }
                    }
                }
            }
        }
    }

    function setMap(_mapSrc){
        console.log("setMap source : " + _mapSrc);
        mapSrc = "file://" + _mapSrc;
        emptyMap.visible = false;
        mapImage.visible = true;
    }

    function setMapPosition(posX, posY, zoom, mapRotation){
        if(zoom > Style.maxZoom)
            zoom = Style.maxZoom;
        else if(zoom < Style.minZoom)
            zoom = Style.minZoom;
        zoomScale.xScale = zoom;
        mapImage.x = posX;
        mapImage.y = posY;
        topViewId.setMapRotation(mapRotation);
    }


    function mapFileChanged(){
        console.log("changed file " + "file:/" + map._mapFile+ " " + mapImage.source);
        mapImage.source = "";
        mapImage.source = "file:/" + map._mapFile;
    }

    function setMessageTop(status, msg){
        topViewId.setMessageTop(status, msg);
    }

    function getMapRotation(){
        return topViewId.mapRotation;
    }
}
