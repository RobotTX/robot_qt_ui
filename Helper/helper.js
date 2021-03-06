
var PointViewType = {
    PERM: 1,
    TEMP: 2,
    HOME: 3,
    PATHPOINT: 4,
    PATHPOINT_START: 5,
    HOME_TEMP: 6,
    PATHPOINT_END: 7,
    PATHPOINT_NEXT : 8,
    PATHPOINT_START_RED : 9,
    PATHPOINT_START_YELLOW : 10,
    PATHPOINT_RED : 11,
    PATHPOINT_RED_START : 12,
    HOME_ORANGE: 13
};

var noGroup = "Default";
//var noGroupChinese = "默认组"
var noGroupChinese = "Default";
var noRobot = "No Robot";
var noRobotChinese = "没有机器人"

function dashLine(ctx, fromX, fromY, toX, toY, pattern) {
    // Our growth rate for our line can be one of the following:
    //   (+,+), (+,-), (-,+), (-,-)
    // Because of this, our algorithm needs to understand if the x-coord and
    // y-coord should be getting smaller or larger and properly cap the values
    // based on (x,y).
    var lt = function (a, b) { return a <= b; };
    var gt = function (a, b) { return a >= b; };
    var capmin = function (a, b) { return Math.min(a, b); };
    var capmax = function (a, b) { return Math.max(a, b); };

    var checkX = { thereYet: gt, cap: capmin };
    var checkY = { thereYet: gt, cap: capmin };

    if (fromY - toY > 0) {
        checkY.thereYet = lt;
        checkY.cap = capmax;
    }
    if (fromX - toX > 0) {
        checkX.thereYet = lt;
        checkX.cap = capmax;
    }

    ctx.moveTo(fromX, fromY);
    var offsetX = fromX;
    var offsetY = fromY;
    var idx = 0, dash = true;
    while (!(checkX.thereYet(offsetX, toX) && checkY.thereYet(offsetY, toY))) {
        var ang = Math.atan2(toY - fromY, toX - fromX);
        var len = pattern[idx];

        offsetX = checkX.cap(toX, offsetX + (Math.cos(ang) * len));
        offsetY = checkY.cap(toY, offsetY + (Math.sin(ang) * len));

        if (dash) ctx.lineTo(offsetX, offsetY);
        else ctx.moveTo(offsetX, offsetY);

        idx = (idx + 1) % pattern.length;
        dash = !dash;
    }
};

function formatName(name){
    return name.replace(/ +/g, ' ').trim();
}
