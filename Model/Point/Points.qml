import QtQuick 2.7
import QtQuick.Controls 2.1

ListModel {
    id: pointModel
    function addGroup(index, name){
        //console.log("Add group " + name);
        pointModel.insert(index, {"_name": name, "_isVisible": false, "_groupName": "",
                              "_x": 0, "_y": 0});
    }

    function addPoint(index, name, isVisible, groupName, x, y){
        //console.log("Add point " + name);
        pointModel.insert(index, {"_name": name, "_isVisible": isVisible, "_groupName": groupName,
                              "_x": x, "_y": y});
    }
}
