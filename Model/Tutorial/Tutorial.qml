import QtQuick 2.7
import QtQuick.Controls 2.1

ListModel {
    id: listmod
    signal updateFile(int index, bool value)

    ListElement {
        feature: "edit_map"
        message: "You are about to edit a map, here is how to proceed...\n\n\t
                * Select a color\n\n\t
                * Select a a shape\n\n\t
                * Select a size\n\n\t
                * Undo (Ctrl+Z) or redo (Ctrl+Y)
                your actions using the arrows\n\n\t
                * Click reset to start from scratch\n\n\t
                * Don't forget to either cancel or save your modifications\n"
        show: true
    }

    ListElement {
        feature: "merge_map"
        message: "You are about to merge two or more maps together, here is how to proceed...\n\n\t
        * Import two or more maps from the robot or from any folder\n\n\t
        * Drag and rotate your maps until you are satisfied\n\n\t
        * Click reset to start from scratch\n\n\t
        * Don't forget to save or cancel your modifications\n"
        show: true
    }

    ListElement {
        feature: "recover_position"
        message: "You are about to recover the position of one or more of your robots. This is how to proceed...\n\n\t
        * Choose a robot and click on \"start recovering the position\"
        * You can also click the map to set your own goals
        * When the position has been recovered this window will automatically close"
        show: true
    }

    ListElement {
        feature: "scan_map"
        message: "You are about to scan the map. This is how to proceed...\n\n\t
        * Use the teleop keys to make the robot\n\n\t
        * Click the map to set goals from the robot"
        show: true
    }

    function hideMessage(_feature){
        console.log("hiding feature " + _feature)
        for(var i = 0; i < count; i++)
            if(get(i).feature === _feature){
                get(i).show = false
                listmod.updateFile(i, false)
            }
    }

    function showMessage(_feature){
        console.log("showing feature " + _feature)
        for(var i = 0; i < count; i++){
            if(get(i).feature === _feature){
                get(i).show = true
                listmod.updateFile(i, true)
            }
        }
    }

    function getMessage(_feature){
        for(var i = 0; i < count; i++){
            if(get(i).feature === _feature)
                return get(i).message
        }
    }

    function isDisplayed(_feature){
        for(var i = 0; i < count; i++){
            if(get(i).feature === _feature)
                return get(i).show
        }
    }

    // NOTE: do not use this function except for initialization as it is not connected to the c++ side to update the file
    function setVisibleMessage(_feature, visible){
        for(var i = 0; i < count; i++){
            if(get(i).feature === _feature)
                get(i).show = visible;
        }
    }
}
