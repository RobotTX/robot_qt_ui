import QtQuick 2.0

ListModel {
    property real batteryWarningThreshold

    signal newHomeSignal(string ip, double homeX, double homeY, int homeOri)
    signal newPathSignal(string ip, string groupName, string pathName)
    signal newNameSignal(string ip, string newName)
    signal deletePathSignal(string ip)
    signal pausePathSignal(string ip)
    signal playPathSignal(string ip)
    signal stopPathSignal(string ip)
    signal visiblePathChanged()
    signal stopScanning(string ip, bool killGobotMove);
    signal robotDc(string ip);
    signal robotConnection(string ip);
    signal setBusy(string ip, bool busy)
    signal setMessageTop(int status, string msg)
    signal activateLaser(string ip, bool activate)

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
            "homeX": 0,
            "homeY": 0,
            "homeOri": 0,
            "laserActivated": false,
            "scanningOnConnection": false,
            "processingCmd": false
        });
        robotConnection(ip);
        setMessageTop(3, "The robot \"" + name + "\" just connected");
    }

    function removeRobot(ip){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
                setMessageTop(0, "The robot \"" + get(i).name + "\" just disconnected");
                remove(i);
            }
        visiblePathChanged();
        robotDc(ip);
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

    function setHome(ip, posX, posY, ori){
        console.log("Set home " + ip + " " + posX + " " + posY + " " + ori);
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
                setProperty(i, "homeX", posX);
                setProperty(i, "homeY", posY);
                setProperty(i, "homeOri", ori);
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
            if(get(i).ip === ip){
                if(Math.abs(get(i).stage) < 10000 && stage >= 9999){
                    setMessageTop(2, "The robot \"" + get(i).name + "\" just completed its path");
                    stage = 0;
                    setProperty(i, "playingPath", false);
                    stopPathSignal(ip);
                } else if(get(i).stage >= 0 && stage < 0)
                    setMessageTop(0, "The robot \"" + get(i).name + "\" is currently stuck in its path to \"" + get(i).pathPoints.get(Math.abs(stage + 1)).pathPointName + "\"");
                setProperty(i, "stage", stage);
            }
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
                        +  get(i).pathName + " : " +  get(i).playingPath + " : " +  get(i).stage + " : " +  get(i).pathPoints.count);
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

    function setLaserActivated(ip, activate){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
                setProperty(i, "laserActivated", activate);
                console.log("laser set");
            }
    }

    function setName(ip, newName){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                setProperty(i, "name", newName);
    }

    function getName(ip){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                return get(i).name;
        return "";
    }

    function setScanningOnConnection(ip, scanning){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                setProperty(i, "scanningOnConnection", scanning);
    }

    function getScanningOnConnection(ip){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                return get(i).scanningOnConnection;
        return false;
    }

    function stopAllScanOnConnection(){
        /// Stop the scan if a scanning robot reconnect after the window has been closed
        for(var i = 0; i < count; i++)
            if(get(i).scanningOnConnection === true)
                stopScanning(get(i).ip, true);
    }

    function isConnected(ip){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                return true;
        return false;
    }

    function setProcessingCmd(ip, processing){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                setProperty(i, "processingCmd", processing);
        setBusy(ip, processing);
    }
}
