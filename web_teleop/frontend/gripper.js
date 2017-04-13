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
      strength: desiredStrength
    });
    setGripperClient.callService(request);
  });

    // Set the Strength when the button is clicked.
  gripperCloseButton.addEventListener('click', function() {
    var request = new ROSLIB.ServiceRequest({
      strength: desiredStrength
    });
    setGripperClient.callService(request);
  });
}