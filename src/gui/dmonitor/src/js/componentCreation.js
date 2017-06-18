var Robot;
var robots = [];
var robotnum = 6;

function createObjects() {
    Robot = Qt.createComponent("../qml/Robot.qml");

    if(Robot.status === Component.Ready)
        finishRobotCreation();
    else
        Robot.statusChanged.connect(finishCreation);

    return robots;
}


function finishRobotCreation() {
   if(Robot.status === Component.Ready) {
       for(var i = 1; i <= robotnum; ++i) {
           var rbt = Robot.createObject(drawArea, {"robotId": i,
                                                   "width": 10,
                                                   "height": 10,
                                                   "address": udpAddress,
                                                   "field": field})
           robots.push(rbt);
       }

       for(i = 0; i < robots.length; ++i) {
           robots[i].init();
       }
   } else if (Robot.status === Component.Error) {
       console.log('Error loading Robot: ', Robot.errorString())
   }
}
