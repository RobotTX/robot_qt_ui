import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../View/Point"
import "../../Helper/helper.js" as Helper

ListModel {
    function addGroup(index, name){
        console.log("Add group " + name);
        insert(index, {"_name": name, "_isVisible": false, "_groupName": "",
                              "_x": 0, "_y": 0});
    }

    function addPoint(index, name, isVisible, groupName, x, y){
        console.log("Add point " + name + " " + x + " " + y);
        insert(index, {"_name": name, "_isVisible": isVisible, "_groupName": groupName,
                              "_x": x, "_y": y});
    }

    function removePoint(index){
        remove(index);
    }

    function removeGroup(begin, end){
        for(var i = begin; i <= end; i++)
            remove(begin);
    }

    function hideShow(index, show){
        setProperty(index, "_isVisible", show);
    }

    function renameGroup(newName, oldName){
        for(var i = 0; i < count; i++){
            if(get(i)._groupName === "" && get(i)._name === oldName)
                get(i)._name = newName;
            else if(get(i)._groupName === oldName)
                get(i)._groupName = newName;
        }
    }

    function movePoint(oldIndex, newIndex, newGroupName){
        displayList();
        console.log("move to " + oldIndex + " " + newIndex + " " + count);
        move(oldIndex, newIndex, 1);
        setProperty(newIndex, "_groupName", newGroupName);
        displayList();
    }

    function displayList(){
        console.log("\nList of points :");
        for(var i = 0; i < count; i++){
            console.log(i + " : " + get(i)._groupName + " " + get(i)._name);
        }
    }
}
