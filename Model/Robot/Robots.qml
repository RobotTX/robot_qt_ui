import QtQuick 2.0
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import "../../View/Custom"

ListModel {
    property real batteryWarningThreshold
    property variant msgs: []
    property variant inverseMsg: []
    property variant tmp: []

    property string msg: ""
    property string test: ""
    property int statusColor: -1
    property string langue
    property bool sendPointToRobot
    property string ipRobot: ""
    property bool stopButtonClicked: false
    property bool pauseButtonClicked: false
    property bool deletePathButtonClicked: false
    property bool pathCompleted: false
    property bool robotStuck: false
    property bool robotSelected: false
    property string pathNameAssigned: ""
    property string nameRobotPath: ""
    property string robotIP: ""

    signal sendTtsToRobot(string ip, string tts)
    signal savePlaceSignal(string ip, string name, double x, double y, double orientation, bool home)
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
    signal setLoopingPathSignal(string ip, bool loop)
    signal saveWifiConnection(string ip, string wifi_name, string pwd_wifi)
    signal soundOn(string ip)
    signal soundOff(string ip)

    function addRobot(name, ip, stage, battery){
        var message = ''
        append({
            "name": name,
            "ip": ip,
            "stage": stage,
            "battery": battery,
            "linearVelocity": 0.4,
            "angularVelocity": 40,
            "posX": -1,
            "posY": -1,
            "orientation": 0,
            "playingPath": false,
            "pathIsOpen": true,
            "pathIsVisible": true,
            "pathName": "",
            "pathPoints": [],
            "homeX": -1,
            "homeY": -1,
            "homeOri": 0,
            "laserActivated": false,
            "scanningOnConnection": false,
            "processingCmd": false,
            "dockStatus": -2,
            "charging": false,
            "looping": false,
            "batteryLevel": 0,
        });
        robotConnection(ip);
        if (langue == "English") {
            message = "The robot \"" + name + "\" just connected";
        } else {
            message = "机器人 \"" + name + "\"已连接";
        }

        setMessageTop(2, message);
    }

    function removeRobot(ip){
        var message = ''
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
                if (langue == "English") {
                    message = "机器人 \"" + get(i).name + "\" 失去连接"
                } else {
                    message = "The robot \"" + get(i).name + "\" just disconnected"
                }

                setMessageTop(0, message);
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

    function setTts(ip, tts) {
        for (var i = 0; i < count; i++) {
            if (get(i).ip === ip) {
                setProperty(i, "tts", tts);
            }
        }
    }

    function setHome(ip, posX, posY, ori){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
                setProperty(i, "homeX", posX);
                setProperty(i, "homeY", posY);
                setProperty(i, "homeOri", ori);
                setProperty(i, "dockStatus", 0);
            }
    }

    function resetHome(ip){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
                setProperty(i, "homeX", 0);
                setProperty(i, "homeY", 0);
                setProperty(i, "homeOri", 0);
                setProperty(i, "dockStatus", -2);
            }
    }

    function setLinearVelocity(ip, linear) {
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
                setProperty(i, "linearVelocity = ", linear);
            }
    }

    function setAngularVelocity(ip, angular) {
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
                setProperty(i, "angularVelocity", angular);
            }
    }

    function setVelocity(ip, linear, angular) {
           for(var i = 0; i < count; i++)
               if(get(i).ip === ip){
                   setProperty(i, "linearVelocity", linear);
                   setProperty(i, "angularVelocity", angular)
               }
       }

   function getLinearVelocity(ip) {
       for (var i = 0; i < count; i++) {
           if (get(i).ip === ip) {
               return get(i).linearVelocity;
           }
       }
       return false;
   }

   function getAngularVelocity(ip) {
       for (var i = 0; i < count; i++) {
           if (get(i).ip === ip) {
               return get(i).angularVelocity;
           }
       }
       return false;
   }

   function setBatteryWarning(ip, batteryLevel) {
           for (var i = 0; i < count; i++) {
               if (get(i).ip === ip) {
                   setProperty(i, "batteryLevel", batteryLevel);
               }
           }
       }

   function getBatteryWarning(ip) {
       for (var i = 0; i < count; i++) {
           if (get(i).ip === ip) {
               return get(i).batteryLevel;
           }
       }
       return false;
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
        var message = ''
        pathCompleted = false;
        robotStuck = false;

        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
                //to change the playing icon after completing path
                if (stage === get(i).pathPoints.count){
                    setProperty(i, "playingPath", false);
                }

                else if(Math.abs(get(i).stage) < 10000 && stage >= 9999){
                    if (langue == "English") {
                        message ="The robot \"" + get(i).name + "\" just completed its path"
                        pathCompleted = true;
                    } else {
                        message = "机器人 " + get(i).name + " 完成路径"
                    }
                    setMessageTop(2, message);
                    stage = 0;
                    setProperty(i, "playingPath", false);
                    if(get(i).dockStatus !== 3)
                        stopPathSignal(ip);
                } else if(get(i).stage >= 0 && stage < 0) {
                    robotStuck = true;
                    if (langue == "English") {
                        message = "机器人 " + get(i).name + " 被阻挡在当前路径 " + get(i).pathPoints.get(Math.abs(stage + 1)).pathPointName + "\"";
                    } else {
                        message ="The robot \"" + get(i).name + "\" is currently stuck in its path to \"" + get(i).pathPoints.get(Math.abs(stage + 1)).pathPointName + "\"";
                    }
                } else {

                }

                setMessageTop(0, message);
                setProperty(i, "stage", stage);
            }
    }

    function setBattery(ip, battery, charging){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
//                setProperty(i, "battery", battery);
                setProperty(i, "battery", battery);
                setProperty(i, "charging", charging);
            }
    }

    function addPathPoint(ip, name, posX, posY, waitTime, orientation, speechName, speechContent, speechTime){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
                get(i).pathPoints.append({
                                          "pathPointName": name,
                                          "pathPointPosX": posX,
                                          "pathPointPosY": posY,
                                          "waitTime": waitTime,
                                          "orientation": orientation,
                                          "speechName": speechName,
                                          "speechContent": speechContent,
                                          "speechTime": speechTime
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
//        for(var i = 0; i < count; i++)
//            console.log(get(i).name + " : " + get(i).ip + " : " + get(i).posX + " : " +  get(i).posY + " : "
//                        +  get(i).pathName + " : " +  get(i).playingPath + " : " +  get(i).stage + " : " +  get(i).pathPoints.count);
    }

    function hideShowPathOnMap(ip){
        for(var i = 0; i < count; i++){
            /// to show one by one the path
//            if(get(i).ip === ip) {
//                setProperty(i, "pathIsVisible", !get(i).pathIsVisible);
//            } else {
//                setProperty(i, "pathIsVisible", false);

//            }
            if(get(i).ip === ip) {
                console.log("we are in hideShowPathOnMap(ip)");
                setProperty(i, "pathIsVisible", !get(i).pathIsVisible);
            }
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
            }
    }

    function setName(ip, newName){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                setProperty(i, "name", newName);
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

    function getName(ip){
        var name = "Default Name";
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip)
                name = get(i).name;
        return name;
    }

    function setLooping(ip, loop){
        for(var i = 0; i < count; i++)
            if(get(i).ip === ip){
                setProperty(i, "looping", loop);
                ipRobot = ip;
            }
    }

    /**
     * -4: the robot could not reach the landing point in front of the station
     * -3: obstacle in the way
     * -2: no home
     * -1: failed to dock
     * 0: not docked
     * 1: docked
     * 2: docked but check alignment
     * 3: in progress
     */
    function setDockStatus(ip, dockStatus){
        /// TODO set message top according to new status
        var message1 = ''
        var message2 = ''
        var message3 = ''
        var message4 = ''
        var message5 = ''
        var message6 = ''

        for(var i = 0; i < count; i++)
            if(get(i).ip === ip && get(i).dockStatus !== dockStatus){
                setProperty(i, "dockStatus", dockStatus);
                if (langue == "English") {
                    message1 = "机器人 " + get(i).name + " 不能到达充电站"
                    message2 = "有障碍阻挡机器人 " + get(i).name + " 到达充电站"
                    message3 = "机器人 " + get(i).name + " 失去充电站的信号因此不能到达充电站"
                    message4 = "机器人 " + get(i).name + " 到达充电站"
                    message5 = "机器人 " + get(i).name + " 到达充电站，但可能没有完全对准充电站"
                    message6 = "机器人 " + get(i).name +" 正在前往充电站"
                } else {
                    message1 = "Robot \"" + get(i).name + "\" could not reach home"
                    message2 = "An obstacle is blocking the robot \"" + get(i).name + "\" to reach its home"
                    message3 = "Robot \"" + get(i).name + "\" lost the signal of its home and could not reach it"
                    message4 = "Robot \"" + get(i).name + "\" successfully reached its home"
                    message5 = "Robot \"" + get(i).name + "\" successfully reached its home but might not be properly aligned"
//                    message6 = "Robot \"" + get(i).name + "\" heading to home"
                }


                /// we are only interested by a change of status
                switch(dockStatus){
                    case -4:
                        setMessageTop(0, message1);
//                        warningDialog.open();
                    break;
                    case -3:
                        setMessageTop(0, message2);
//                        warningDialog.open();
                    break;
                    case -1:
                        setMessageTop(0, message3);
//                        warningDialog.open();
                    break;
                    case 1:
                        setMessageTop(2, message4);
                    break;
                    case 2:
                        setMessageTop(2, message5);
                    break;
//                    case 3:
//                        setMessageTop(2, message6);
//                    break;
                }
            }
    }

    function setSound(ip, mute) {
        for (var i = 0; i < count; i++) {
            if (get(i).ip === ip) {
                setProperty(i,"charging", mute);
//                console.log("mute = " + mute)
            }
        }

//        console.log("we are in setSound in Robot.qml file mute = " + mute)
    }

    function reverse(arr1, arr2, len) {
        for (var i = len-1; i >=0; i--) {
            arr2[(len-1) - i] = arr1[i];
        }
    }

    function getPathName() {
        return pathNameAssigned;
    }

    function getRobotNamePath() {
        return nameRobotPath;
    }

    function getRobotIP() {
        return robotIP;
    }
}
