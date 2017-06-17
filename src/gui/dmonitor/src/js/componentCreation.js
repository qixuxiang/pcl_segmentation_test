var component;
var robots = [];

function createObjects() {
    console.log("Create !");
    component = Robot;
    if(component.status === Component.Ready)
        finishCreation();
    else
        component.statusChanged.connect(finishCreation);
}


function finishCreation() {
   if(component.status === Component.Ready) {
       for(var i = 1; i <= 6; ++i) {
           var rbt = component.createObject(Robot, {"robotId": i, "width": 10, "height": 10, "address": "192.168.255.255"})
           robots.push(rbt);
       }

       for(i = 0; i < robots.length; ++i) {
           robots[i].init();
       }
   } else if (component.status === Component.Error) {
       // Error Handling
       console.log('Error loading component: ', component.errorString())
   }
}


function updateRobot() {
    for(var i = 0; i < robots.length; ++i) {
        robots[i].update();
    }
}
