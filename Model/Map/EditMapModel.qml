import QtQuick 2.0

ListModel {
    // update is a boolean, if true then we don't append a new item to the list we simply update the last element
    function addItem(shape, thickness, color, points, update){

        if(!update){
//            console.log("Add Item " + shape + " number points " + points.length / 2);
            append({
               "shape": shape,
               "thickness": thickness,
               "color": color,
               "points": []
            });
        }

        for(var i = 0; i < points.length; i += 2)
            addPointToGroup(points[i], points[i+1]);
    }

    function deleteItem(name){
        for(var i = 0; i < count; i++)
            if(get(i).id === id)
                get(i).points = [];
    }

    function addPointToGroup(x, y){
        if(count > 0)
            get(count-1).points.append({
                                 "x": x,
                                 "y": y
                                 });
    }
}
