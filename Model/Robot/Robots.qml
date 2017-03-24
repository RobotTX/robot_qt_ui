import QtQuick 2.0

ListModel {

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
            "playingPath": 0,
            "homeName": "",
            "homeX": 0,
            "homeY": 0,
            "pathName": "",
            "pathPoints": []
        });
    }

    function removeRobot(ip){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                remove(i);
        console.log("Remove robot " + count);
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
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
                get(i).pathPoints.append({
                                          "name": name,
                                          "posX": posX,
                                          "posY": posY,
                                          "waitTime": waitTime
                                      });
                console.log(ip + " addPathPoint " + get(i).pathPoints.count);
            }
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
}
