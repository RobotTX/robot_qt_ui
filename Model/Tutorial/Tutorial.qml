import QtQuick 2.7
import QtQuick.Controls 2.1

ListModel {
    id: listmod
    signal updateFile(int index, bool value)
    property string langue

    ListElement {
        feature: "EDIT MAP"
        message:"You are about to edit a map, here is how to proceed...\n\t
* Select a color\n\t
* Select a a shape\n\t
* Select a size\n\t
* Undo (Ctrl+Z) or redo (Ctrl+Y) your actions using the arrows\n\t
* Click reset to start from scratch\n\t
* Don't forget to either cancel or save your modifications\n"
        show: true
    }

    ListElement {
        feature: "修改地图"
        message: " 请阅读以下信息以便修改地图:\n\t
* 选择颜色 \n\t
* 选择形状 \n\t
* 选择尺寸 \n\t
* 撤销 (Ctrl+Z) 或者 恢复 (Ctrl+Y) 操作 \n\t
* 单击重置来撤销所有修改 \n\t
* 选择取消或者保存结束修改地图 \n"
        show: true
    }

    ListElement {
        feature: "recover_position_chinese"
        message: "你即将恢复机器人的一个或多个位置。处理过程 ...\n\t
* 选择一个机器人并点击\"开始恢复位置\"\n]t
* 你可以点击地图设置自己的目标\n\t
* 当位置恢复后窗口会自动关闭\n"
        show: true
    }

    ListElement {
        feature: "recover_position"
        message: "You are about to recover the position of one or more of your robots. This is how to proceed...\n\t
* Choose a robot and click on \"start recovering the position\"\n\t
* You can also click the map to set your own goals\n\t
* When the position has been recovered this window will automatically close\n"
        show: true
    }

    ListElement {
        feature: "SCAN MAP"
        message: "You are about to scan a map. This is how to proceed...\n
* Use the teleop buttons or your keyboard (see touches below) to make the robot move\n\t
\tu i o \n\t
\tj k l \n\t
\tm , . \n\t
* Click the map to set goals from the robot"
        show: true
    }

    ListElement {
        feature: "扫描地图"
        message: "请阅读以下信息以便扫描地图: \n\t
* 使用相应键盘按键(请看以下提示)来移动机器人 \n\t
\tu i o \n\t
\tj k l \n\t
\tm , . \n\t
* 点击地图为机器人设置目标点"
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
