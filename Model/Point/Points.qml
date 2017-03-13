import QtQuick 2.7
import QtQuick.Controls 2.1
import "../../View/Point"
import "../../Helper/helper.js" as Helper

ListModel {
    id: pointModel

    function addGroup(index, name){
        console.log("Add group " + name);
        pointModel.insert(index, {"_name": name, "_isVisible": false, "_groupName": "",
                              "_x": 0, "_y": 0});
    }

    function addPoint(index, name, isVisible, groupName, x, y){
        console.log("Add point " + name + " " + x + " " + y);
        pointModel.insert(index, {"_name": name, "_isVisible": isVisible, "_groupName": groupName,
                              "_x": x, "_y": y});
    }

    function removePoint(index){
        pointModel.remove(index);
    }

    function removeGroup(begin, end){
        for(var i = begin; i <= end; i++)
            pointModel.remove(begin);
    }

    function hideShow(index, show){
        //console.log("Modifying the visible property at index " + index + " " + pointModel.get(index)._name + " to " + show);
        pointModel.setProperty(index, "_isVisible", show);
    }
}
