import QtQuick 2.0

ListModel {
    signal newHomeSignal(string ip, string homeName, double homeX, double homeY)
    signal newPathSignal(string ip, string groupName, string pathName)
    signal newNameSignal(string ip, string newName)
    signal deletePathSignal(string ip)
    signal visiblePathChanged()

    function addRobot(name, ip, wifi, stage, battery){
        append({
            "name": name,
            "ip": ip,
            "wifi": wifi,
            "stage": stage,
            "battery": battery,
            "posX": -1,
            "posY": -1,
            "orientation": 0,
            "playingPath": false,
            "pathIsOpen": false,
            "pathIsVisible": false,
            "pathName": "",
            "pathPoints": [],
            "homeName": "",
            "homeX": 0,
            "homeY": 0,
            "laserActivated": false
        });
    }

    function removeRobot(ip){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                remove(i);
        //console.log("Remove robot " + count);
    }

    function setPos(ip, posX, posY, orientation){
        for(var i = 0; i < count; i++){
            if(get(i).ip === ip){
                setProperty(i, "posX", posX);
                setProperty(i, "posY", posY);
                setProperty(i, "orientation", orientation);
            }
        }
    }

    function setHome(ip, name, posX, posY){
        console.log("Set home " + ip + " " + name + " " + posX + " " + posY);
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
                setProperty(i, "homeName", name);
                setProperty(i, "homeX", posX);
                setProperty(i, "homeY", posY);
            }
    }

    function setPath(ip, name){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
                setProperty(i, "pathName", name);
                get(i).pathPoints.clear();
            }
        visiblePathChanged();
    }

    function setStage(ip, stage){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                setProperty(i, "stage", stage);
    }

    function setBattery(ip, battery){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                setProperty(i, "battery", battery);
    }

    function addPathPoint(ip, name, posX, posY, waitTime){
        console.log("addPathPoint " + ip + " " + name + " " + posX + " " + posY + " " + waitTime);
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
                get(i).pathPoints.append({
                                          "pathPointName": name,
                                          "pathPointPosX": posX,
                                          "pathPointPosY": posY,
                                          "waitTime": waitTime
                                      });
            }
        visiblePathChanged();
    }

    function setPlayingPath(ip, playingPath){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                setProperty(i, "playingPath", playingPath);
    }

    function display(){
        console.log("Displaying the robots :\n");
        for(var i = 0; i < count; i++)
            console.log(get(i).name + " : " + get(i).ip + " : " + get(i).posX + " : " +  get(i).posY + " : "
                        +  get(i).homeName + " : " +  get(i).pathName + " : " +  get(i).playingPath + " : " +  get(i).stage + " : " +  get(i).pathPoints.count);
    }

    function hideShowPathOnMap(ip){
        for(var i = 0; i < count; i++){
            if(get(i).ip === ip)
                setProperty(i, "pathIsVisible", !get(i).pathIsVisible);
            else
                setProperty(i, "pathIsVisible", false);
        }
    }

    function openPath(ip){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                setProperty(i, "pathIsOpen", !get(i).pathIsOpen);
    }

    function activateLaser(ip){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                setProperty(i, "laserActivated", !get(i).laserActivated);
    }

    function setName(ip, newName){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                setProperty(i, "name", newName);
    }
}
