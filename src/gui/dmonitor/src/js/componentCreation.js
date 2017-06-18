var Robot;
var robots = [];
var udpAddress = "192.168.255.255";

function createObjects() {
    console.log("Create ");
    Robot = Qt.createComponent("../qml/Robot.qml");

    if(Robot.status === Component.Ready)
        finishRobotCreation();
    else
        Robot.statusChanged.connect(finishCreation);

    return robots;
}


function finishRobotCreation() {
   if(Robot.status === Component.Ready) {
       for(var i = 1; i <= 6; ++i) {
           var rbt = Robot.createObject(drawArea, {"robotId": i,
                                                   "width": 10,
                                                   "height": 10,
                                                   "address": udpAddress})
           robots.push(rbt);
           rbt.x = Math.floor(Math.random() * 900)
           rbt.y = Math.floor(Math.random() * 600)
       }

       for(i = 0; i < robots.length; ++i) {
           robots[i].init();
       }
   } else if (Robot.status === Component.Error) {
       console.log('Error loading Robot: ', Robot.errorString())
   }
}
