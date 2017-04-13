Head = function(ros) {
  // HTML elements
  // To get an element with an ID of "baseForward", query it as shown below.
  // Note that any IDs you set on HTML elements should be unique.
  var headForward = document.querySelector('#headForward');
  var headBackward = document.querySelector('#headBackward');
  var headLeft = document.querySelector('#headLeft');
  var headRight = document.querySelector('#headRight');

  var that = this;

  // Public variables
  this.yawSpeed = 0.25;
  this.pitchSpeed = 0.25;

  // Set up the publisher.
  var cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: 'head_controller/follow_joint_trajectory',
    messageType: 'trajectory_msgs/JointTrajectory'
  });

  // Internal function to send a velocity command.
  var move = function(linear, angular) {
    var twist = new ROSLIB.Message({
      linear: {
        x: linear,
        y: 0,
        z: 0
      },
      angular: {
        x: 0,
        y: 0,
        z: angular
      }
    });  
    cmdVel.publish(twist);
  }

  // Handler for when the mouse is held on the up arrow.
  // Instead of writing a loop (which will block the web page), we use
  // setInterval, which repeatedly calls the given function at a given
  // time interval. In this case, it repeatedly calls move() every 50 ms.
  // Note that inside of move, we use that._timer and that.linearSpeed.
  // At the top of the file we set "var that = this" to ensure that the
  // local variable "that" always refers to this Base instance.
  this.moveUp = function() {
    that._timer = setInterval(function() {
      move(0, that.pitchSpeed)
    }, 50);
  }

  this.moveDown = function() {
    that._timer = setInterval(function() {
      move(0, -that.pitchSpeed)
    }, 50);
  }

  this.moveLeft = function() {
    that._timer = setInterval(function() {
      move(that.yawSpeed, 0)
    }, 50);
  }

  this.moveRight = function() {
    that._timer = setInterval(function() {
      move(-that.yawSpeed, 0)
    }, 50);
  }

  // Stops the robot from moving.
  this.stop = function() {
    if (that._timer) {
      clearInterval(that._timer);
    }
    move(0, 0);
  };  

  headUp.addEventListener('mousedown', that.moveUp);
  headDown.addEventListener('mousedown', that.moveDown);
  headLeft.addEventListener('mousedown', that.moveLeft);
  headRight.addEventListener('mousedown', that.moveRight);
  
  // We bind stop() to whenever the mouse is lifted up anywhere on the webpage
  // for safety reasons. We want to be conservative about sending movement commands.
  document.addEventListener('mouseup', that.stop);
}