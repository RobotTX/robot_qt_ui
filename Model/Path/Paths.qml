import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../Helper/helper.js" as Helper

ListModel {

    signal deletePathSignal(string groupName, string name)
    signal deleteGroupSignal(string groupName)
    signal moveToSignal(string name, string oldGroup, string newGroup)
    signal checkTmpPosition(int index, double x, double y)
    signal visiblePathChanged()
    signal validPositionChanged()
    signal setMessageTop(int status, string msg)
    signal saveCurrentPath(string pathName, ListModel pathPoints)
    signal robotConnection(string ip)

    property bool openGroup: false

    property string langue
    property int _index

    function addGroup(name){
//        console.log("paths adding group " + name)
        append({
           "groupName": name,
           "groupIsOpen": langue == "English" ? name === Helper.noGroup : name === Helper.noGroupChinese,
           "paths": []
        });
    }

    function languageChoice(language) {
        if (language === 0) {
            langue = "中文"
        } else {
            langue = "English";

        }
//        console.log("language in paths = " + langue);
    }

    function addPath(name, groupName){
        //console.log("Add path " + name + " to group " + groupName);
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                get(i).paths.append({
                     "pathName": name,
                     "pathIsOpen": false,
                     "pathIsVisible": false,
                     "pathPoints": []
                });
    }

    function addPathPoint(name, pathName, groupName, x, y, waitTime, orientation, speechName, speechContent, speechTime){
        //console.log("Add pathpoint " + name + " to path " + pathName + " in group " + groupName);
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).paths.count; j++)
                    if(get(i).paths.get(j).pathName === pathName){
                        if(get(i).paths.get(j).pathPoints.count > 0){
                            // to prevent the same path point to be added twice in a row
                            if(get(i).paths.get(j).pathPoints.get(get(i).paths.get(j).pathPoints.count - 1).posX !== x
                                    || get(i).paths.get(j).pathPoints.get(get(i).paths.get(j).pathPoints.count - 1).posY !== y
                                    || get(i).paths.get(j).pathPoints.get(get(i).paths.get(j).pathPoints.count - 1).orientation !== orientation)
                                get(i).paths.get(j).pathPoints.append({
                                     "name": name,
                                     "posX": x,
                                     "posY": y,
                                     "waitTime": waitTime,
                                     "validPos": false,
                                     "orientation": orientation,
                                     "speechName": speechName,
                                     "speechContent": speechContent,
                                     "speechTime": speechTime
                                });
                        } else {
                            get(i).paths.get(j).pathPoints.append({
                                 "name": name,
                                 "posX": x,
                                 "posY": y,
                                 "waitTime": waitTime,
                                 "validPos": false,
                                 "orientation": orientation,
                                 "speechName": speechName,
                                 "speechContent": speechContent,
                                 "speechTime": speechTime

                            });
                        }
                    }

    }

    function deleteGroup(groupName){
        var message = ''
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName){
                remove(i);
                if (langue == 'English') {
                    message = "Deleted the group \"" + groupName + "\""
                } else {
                    message = "删除群组 \"" + groupName + "\""
                }
                setMessageTop(3, message);
            }
        deleteGroupSignal(groupName);
        visiblePathChanged();
    }

    function deletePath(groupName, name){
        var message = '';
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).paths.count; j++)
                    if(get(i).paths.get(j).pathName === name){
                        get(i).paths.remove(j);
                        if (langue == 'English') {
//                            message = "在 \"" + name + "\" 删除路径 \"" + groupName + "\""
                            if (groupName === Helper.noGroup) {
                                message = "Deleted the path \"" + name + "\" in \"" + Helper.noGroup + "\""

                            } else {
                                message = "Deleted the path \"" + name + "\" in \"" + groupName + "\""
                            }
                        } else {
                                message = "在 \"" + name + "\" 删除路径 \"" + groupName + "\""
                        }

                        setMessageTop(3, message);
                    }

        deletePathSignal(groupName, name);
        visiblePathChanged();
    }

    function hideShowGroup(groupName){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName) {
                setProperty(i, "groupIsOpen", !get(i).groupIsOpen);
            }
    }

    function hideShowGroupAll(){
        for(var i = 0; i < count; i++)
            setProperty(i, "groupIsOpen", false);
    }

    function showGroupDefault() {
        for (var i = 0; i < count; i++) {
            if (get(i).groupName === Helper.noGroup) {
                setProperty(i, "groupIsOpen", true);
            }
        }
    }

    function hideShowPath(groupName, name){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).paths.count; j++)
                    if(get(i).paths.get(j).pathName === name)
                        get(i).paths.setProperty(j, "pathIsOpen", !get(i).paths.get(j).pathIsOpen);
    }

    function hideShowPathOnMap(groupName, name){
        for(var i = 0; i < count; i++)
            for(var j = 0; j < get(i).paths.count; j++){
                if(get(i).groupName === groupName && get(i).paths.get(j).pathName === name) {
                    get(i).paths.setProperty(j, "pathIsVisible", !get(i).paths.get(j).pathIsVisible);

                } else {
                    get(i).paths.setProperty(j, "pathIsVisible", false);
                }
                visiblePathChanged();
            }
     }

    function setPathVisible() {
        for(var i = 0; i < count; i++)
            for(var j = 0; j < get(i).paths.count; j++){
                    get(i).paths.setProperty(j, "pathIsVisible", false);
            }
    }

    function renameGroup(newName, oldName){
        for(var i = 0; i < count; i++){
            if(get(i).groupName === oldName)
                setProperty(i, "groupName", newName);
        }
    }

    function moveTo(name, oldGroup, newGroup){
//        console.log("moveTo " + name + " " + oldGroup + " " + newGroup);
        var path = {};
        var pathPoints = [];
        for(var i = 0; i < count; i++){
            if(get(i).groupName === oldGroup){
                for(var j = 0; j < get(i).paths.count; j++){
                    if(get(i).paths.get(j).pathName === name){
                        for(var k = 0; k < get(i).paths.get(j).pathPoints.count; k++){
                            pathPoints.push({
                                "name": get(i).paths.get(j).pathPoints.get(k).name,
                                "posX": get(i).paths.get(j).pathPoints.get(k).posX,
                                "posY": get(i).paths.get(j).pathPoints.get(k).posY,
                                "waitTime": get(i).paths.get(j).pathPoints.get(k).waitTime,
                                "validPos": get(i).paths.get(j).pathPoints.get(k).validPos,
                                "orientation": get(i).paths.get(j).pathPoints.get(k).orientation,
                                "speechName": get(i).paths.get(j).pathPoints.get(k).speechName,
                                "speechContent": get(i).paths.get(j).pathPoints.get(k).speechContent,
                                "speechTime": get(i).paths.get(j).pathPoints.get(k).speechTime
                           });
                        }

                        path = {
                            "pathName": get(i).paths.get(j).pathName,
                            "pathIsOpen": get(i).paths.get(j).pathIsOpen,
                            "pathIsVisible": get(i).paths.get(j).pathIsVisible,
                            "pathPoints": pathPoints
                        }
                        get(i).paths.remove(j);
                    }
                }
            }
        }

        var message = ''
        for(i = 0; i < count; i++) {
            if(get(i).groupName === newGroup) {
                get(i).paths.append(path);
                 if (langue == 'English') {
                    message = "Moved the path \"" + name + "\" from \"" + oldGroup + "\" to \"" + newGroup + "\""
                } else {
                    message = "移动路径 \"" + name + "\" 从 \"" + oldGroup + "\" 到 \"" + newGroup + "\""
                }
            }
        }
        setMessageTop(3, message);
        moveToSignal(name, oldGroup, newGroup)
    }

    function clearTmpPath(){
        clear();
        addGroup("tmpGroup");
        addPath("tmpPath", "tmpGroup");
        hideShowPathOnMap("tmpGroup", "tmpPath");
    }

    function setTmpValidPosition(index, valid){
        get(0).paths.get(0).pathPoints.setProperty(index, "validPos", valid);
        validPositionChanged();
    }

    function deleteAllPaths(){
        clear();
        addGroup(Helper.noGroup);
    }

    function setOrientation(groupName, pathName, index, orientation){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).paths.count; j++)
                    if(get(i).paths.get(j).pathName === pathName)
                        get(i).paths.get(j).pathPoints.get(index).orientation = orientation;
    }

    function setSpeechInfos(groupName, pathName, index, speechName, speechContent){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).paths.count; j++)
                    if(get(i).paths.get(j).pathName === pathName) {
                        get(i).paths.get(j).pathPoints.get(index).speechName = speechName;
                        get(i).paths.get(j).pathPoints.get(index).speechContent = speechContent;

                    }
    }
}
