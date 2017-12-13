import QtQuick 2.7
import QtQuick.Controls 2.1

ListModel {
    id: listmod
    signal updateFile(int index, bool value)
    property string langue

    property string chinese_vers_1: " 请阅读以下信息以便修改地图:\n\n\t
                                       * 选择颜色 \n\n\t
                                       * 选择形状 \n\n\t
                                       * 选择尺寸 \n\n\t
                                       * 撤销 (Ctrl+Z) 或者 恢复 (Ctrl+Y) 操作 \n\n\t
                                       * 单击重置来撤销所有修改 \n\n\t
                                       * 选择取消或者保存结束修改地图 \n"

    property string english_vers_1: "You are about to edit a map, here is how to proceed...\n\n\t
                * Select a color\n\n\t
                * Select a a shape\n\n\t
                * Select a size\n\n\t
                * Undo (Ctrl+Z) or redo (Ctrl+Y)
                your actions using the arrows\n\n\t
                * Click reset to start from scratch\n\n\t
                * Don't forget to either cancel or save your modifications\n"
    ListElement {
        feature: "edit_map"
//        message: langue == "English" ? " 请阅读以下信息以便修改地图:\n\n\t
//                                       * 选择颜色 \n\n\t
//                                       * 选择形状 \n\n\t
//                                       * 选择尺寸 \n\n\t
//                                       * 撤销 (Ctrl+Z) 或者 恢复 (Ctrl+Y) 操作 \n\n\t
//                                       * 单击重置来撤销所有修改 \n\n\t
//                                       * 选择取消或者保存结束修改地图 \n":
//                "You are about to edit a map, here is how to proceed...\n\n\t
//                * Select a color\n\n\t
//                * Select a a shape\n\n\t
//                * Select a size\n\n\t
//                * Undo (Ctrl+Z) or redo (Ctrl+Y)
//                your actions using the arrows\n\n\t
//                * Click reset to start from scratch\n\n\t
//                * Don't forget to either cancel or save your modifications\n"
        message: "test"
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
//        message: langue == "English" ? "请阅读以下信息以便扫描地图: \n\n\t
//        * 使用相应键盘按键(请看以下提示)来移动机器人 \n\t
//        \tu i o \n\t
//        \tj k l \n\t
//        \tm , . \n\n\t
//        * 点击地图为机器人设置目标点"  :
//        "You are about to scan the map. This is how to proceed...\n\n\t
//        * Use the teleop buttons or your keyboard (see touches below) to make the robot move\n\t
//        \tu i o \n\t
//        \tj k l \n\t
//        \tm , . \n\n\t
//        * Click the map to set goals from the robot"
        message: "test"
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
