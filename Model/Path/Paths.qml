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

    function addGroup(name){
        append({
           "groupName": name,
           "groupIsOpen": name === Helper.noGroup,
           "paths": []
        });
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

    function addPathPoint(name, pathName, groupName, x, y, waitTime){
        //console.log("Add pathpoint " + name + " to path " + pathName + " in group " + groupName);
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).paths.count; j++)
                    if(get(i).paths.get(j).pathName === pathName){
                        if(get(i).paths.get(j).pathPoints.count > 0){
                            // to prevent the same path point to be added twice in a row
                            if(get(i).paths.get(j).pathPoints.get(get(i).paths.get(j).pathPoints.count - 1).posX !== x
                                    || get(i).paths.get(j).pathPoints.get(get(i).paths.get(j).pathPoints.count - 1).posY !== y)
                                get(i).paths.get(j).pathPoints.append({
                                     "name": name,
                                     "posX": x,
                                     "posY": y,
                                     "waitTime": waitTime,
                                     "validPos": false
                                });
                        } else {
                            get(i).paths.get(j).pathPoints.append({
                                 "name": name,
                                 "posX": x,
                                 "posY": y,
                                 "waitTime": waitTime,
                                 "validPos": false
                            });
                        }
                    }

    }

    function deleteGroup(groupName){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                remove(i);
        deleteGroupSignal(groupName);
        visiblePathChanged();
    }

    function deletePath(groupName, name){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                for(var j = 0; j < get(i).paths.count; j++)
                    if(get(i).paths.get(j).pathName === name)
                        get(i).paths.remove(j);

        deletePathSignal(groupName, name);
        visiblePathChanged();
    }

    function hideShowGroup(groupName){
        for(var i = 0; i < count; i++)
            if(get(i).groupName === groupName)
                setProperty(i, "groupIsOpen", !get(i).groupIsOpen);
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
                if(get(i).groupName === groupName && get(i).paths.get(j).pathName === name)
                    get(i).paths.setProperty(j, "pathIsVisible", !get(i).paths.get(j).pathIsVisible);
                else
                    get(i).paths.setProperty(j, "pathIsVisible", false);
                visiblePathChanged();
            }
     }

    function renameGroup(newName, oldName){
        for(var i = 0; i < count; i++){
            if(get(i).groupName === oldName)
                setProperty(i, "groupName", newName);
        }
    }

    function moveTo(name, oldGroup, newGroup){
        console.log("moveTo " + name + " " + oldGroup + " " + newGroup);
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
                                "validPos": get(i).paths.get(j).pathPoints.get(k).validPos
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

        for(i = 0; i < count; i++)
            if(get(i).groupName === newGroup)
                get(i).paths.append(path);

        moveToSignal(name, oldGroup, newGroup)
    }

    function clearTmpPath(){
        clear();
        addGroup("tmpGroup");
        addPath("tmpPath", "tmpGroup");
        hideShowPathOnMap("tmpGroup", "tmpPath");
    }

    function setTmpValidPosition(index, error){
        get(0).paths.get(0).pathPoints.setProperty(index, "validPos", !error);
        validPositionChanged();
    }

    function deleteAllPaths(){
        clear();
    }
}
