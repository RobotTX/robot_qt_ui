import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../View/Point"
import "../../Helper/helper.js" as Helper

ListModel {
    id: listModel
    function addGroup(index, name){
        console.log("Add group " + name);
        insert(index, {"_name": name, "_isVisible": true, "_groupName": "",
                              "_x": 0, "_y": 0, "_groupIsOpen": (name === Helper.noGroup) ? true : false});
    }

    function addPoint(index, name, isVisible, groupName, x, y){
        console.log("Add point " + name + " to group " + groupName + " " + x + " " + y);
        var groupIsOpen = Helper.groupIsOpen(listModel, groupName);
        insert(index, {"_name": name, "_isVisible": isVisible, "_groupName": groupName,
                              "_x": x, "_y": y, "_groupIsOpen": groupIsOpen});
    }

    function removePoint(index){
        remove(index);
    }

    function removeGroup(begin, end){
        for(var i = begin; i <= end; i++)
            remove(begin);
    }

    function hideShow(index, show){
        console.log("hideShow " + index + " " + get(index)._groupName + " " + get(index)._name + " " + get(index)._isVisible + " " + show);
        if(get(index)._groupName === ""){
            setProperty(index, "_groupIsOpen", show);
            for(var i = 0; i < count; i++)
                if(get(i)._groupName === get(index)._name)
                    get(i)._groupIsOpen = show;

        } else
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
        console.log("move to " + oldIndex + " " + newIndex+ " " + count + " " + newGroupName );
        setProperty(oldIndex, "_groupName", newGroupName);
        move(oldIndex, newIndex, 1);
    }

    function displayList(){
        console.log("\nList of points :");
        for(var i = 0; i < count; i++){
            console.log(i + " : " + get(i)._groupName + " " + get(i)._name);
        }
    }
}
