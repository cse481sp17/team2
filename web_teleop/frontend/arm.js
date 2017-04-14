Arm = function(ros) {
  // HTML elements
  var armSlider1 = document.querySelector('#armSlider1');
  var armSlider2 = document.querySelector('#armSlider2');
  var armSlider3 = document.querySelector('#armSlider3');
  var armSlider4 = document.querySelector('#armSlider4');
  var armSlider5 = document.querySelector('#armSlider5');
  var armSlider6 = document.querySelector('#armSlider6');
  var armSlider7 = document.querySelector('#armSlider7');
  var armButton = document.querySelector('#armButton');

  var that = this;

  var setArmClient = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_arm',
    serviceType: 'web_teleop/SetArm'
  });

  // Initialize slider.
  var desiredPos1 = 0.0;
  var desiredPos2 = 0.0;
  var desiredPos3 = 0.0;
  var desiredPos4 = 0.0;
  var desiredPos5 = 0.0;
  var desiredPos6 = 0.0;
  var desiredPos7 = 0.0;

  // For most input elements, the .value field is both a getter and a setter.
  // Here we can set its value to the default (0.1).
  armSlider1.value = desiredPos1;
  armSlider2.value = desiredPos2;
  armSlider3.value = desiredPos3;
  armSlider4.value = desiredPos4;
  armSlider5.value = desiredPos5;
  armSlider6.value = desiredPos6;
  armSlider7.value = desiredPos7;

  // Update desiredHeight when slider moves.
  armSlider1.addEventListener('input', function() {
    desiredPos1 = armSlider1.value;
  });
  armSlider2.addEventListener('input', function() {
    desiredPos2 = armSlider2.value;
  });
  armSlider3.addEventListener('input', function() {
    desiredPos3 = armSlider3.value;
  });
  armSlider4.addEventListener('input', function() {
    desiredPos4 = armSlider4.value;
  });
  armSlider5.addEventListener('input', function() {
    desiredPos5 = armSlider5.value;
  });
  armSlider6.addEventListener('input', function() {
    desiredPos6 = armSlider6.value;
  });
  armSlider7.addEventListener('input', function() {
    desiredPos7 = armSlider7.value;
  });

  // Method to set the height.
  this.setArm = function(arm_joints) {
    var request = new ROSLIB.ServiceRequest({
      arm_joints: arm_joints
    });
    setArmClient.callService(request);
  };

  // Set the height when the button is clicked.
  armButton.addEventListener('click', function() {
    console.log("Button clicked!");
    that.setArm([desiredPos1, desiredPos2, desiredPos3, desiredPos4, desiredPos5, desiredPos6, desiredPos7]);
  });
}
