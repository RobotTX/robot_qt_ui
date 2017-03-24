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
               "home": {
                       "name": "",
                       "posX": -1,
                       "posY": -1
                   },
               "path": {
                       "name": "",
                       "pathPoints": []
                   }
           });
    }

    function removeRobot(ip){
        for(var i = 0; i < count; i++){
            if(get(i).ip === ip){
                remove(i);
            }
        }
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
        for(var i = 0; i < count; i++){
            if(get(i).ip === ip){
                setProperty(i, "home", {
                                "name": name,
                                "posX": posX,
                                "posY": posY
                            });
            }
        }
    }

    function setPath(ip, name){
        for(var i = 0; i < count; i++){
            if(get(i).ip === ip){
                setProperty(i, "path", {
                                "name": name,
                                "pathPoints": []
                            });
            }
        }
    }

    function addPathPoint(ip, name, posX, posY, waitTime){
        for(var i = 0; i < count; i++){
            if(get(i).ip === ip){
                get(i).path.pathPoints.push({
                                          "name": name,
                                          "posX": posX,
                                          "posY": posY,
                                          "waitTime": waitTime
                                      });
            }
        }
    }

    function display(){
        console.log("Displaying the robots :\n");
        for(var i = 0; i < count; i++){
            console.log(get(i).name + " : " + get(i).ip + " : " + get(i).posX + " : " +  get(i).posY + " : " +  get(i).home.name + " : " +  get(i).path.name);
        }
    }
}
