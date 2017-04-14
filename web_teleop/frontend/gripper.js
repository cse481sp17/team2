Gripper = function(ros) {
  // HTML elements  
  var desiredGripperStrength = document.querySelector('#desiredGripperStrength');
  var gripperSlider = document.querySelector('#gripperSlider');
  var gripperOpenButton = document.querySelector('#gripperOpenButton');
  var gripperCloseButton = document.querySelector('#gripperCloseButton');

  var that = this;

  var setGripperClient = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_gripper',
    serviceType: 'web_teleop/SetGripper'
  });

  /*
  var gripActionClient = new ROSLIB.ActionClient({
    ros : ros,
      serverName: '/gripper_controller/gripper_action',
      actionName: 'control_msgs/GripperCommandAction'
  });

  var sendGoal = function(pos, strength) {
    var goal = new ROSLIB.Goal({
        actionClient: gripActionClient,
        goalMessage : {
          position: pos,        
          max_effort: strength
        }
    });
    goal.send();
  };
  
  gripperOpenButton.addEventListener('click', function() {
    sendGoal(0.10, desiredStrength); 
  });

  gripperCloseButton.addEventListener('click', function() {
     sendGoal(0.10, desiredStrength); 
  });
  */
 
  // Initialize slider.
  var desiredStrength = 35;
  desiredGripperStrength.textContent = desiredStrength;
  // For most input elements, the .value field is both a getter and a setter.
  // Here we can set its value to the default (0.1).
  gripperSlider.value = desiredStrength;

  // Update desiredStrength when slider moves.
  gripperSlider.addEventListener('input', function() {
    // Read where the slider is now.
    desiredStrength = gripperSlider.value;
    // Update the desired gripper Strength display.
    desiredGripperStrength.textContent = desiredStrength;
  });

  // Set the Strength when the button is clicked.
  gripperOpenButton.addEventListener('click', function() {
    var request = new ROSLIB.ServiceRequest({
      position: 0.10,
      max_effort: parseFloat(desiredStrength)
    });
    setGripperClient.callService(request);
  });

  // Set the Strength when the button is clicked.
  gripperCloseButton.addEventListener('click', function() {
    var request = new ROSLIB.ServiceRequest({
      position: 0,
      max_effort: parseFloat(desiredStrength)
    });
    setGripperClient.callService(request);
  });
}
