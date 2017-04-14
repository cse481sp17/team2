Head = function(ros) {
  // HTML elements
  // To get an element with an ID of "baseForward", query it as shown below.
  // Note that any IDs you set on HTML elements should be unique.
  var headUp = document.querySelector('#headUp');
  var headDown = document.querySelector('#headDown');
  var headLeft = document.querySelector('#headLeft');
  var headRight = document.querySelector('#headRight');

  var that = this;

  // Public variables
  this.panA = 0;
  this.tiltA = 0;

  var setHeadClient = new ROSLIB.Service({
    ros: ros,
    name:'/web_teleop/set_head',
    serviceType: 'web_teleop/SetHead'
  });

  var move = function(pan_, tilt_) {
    var request = new ROSLIB.ServiceRequest({
        pan : parseFloat(pan_),
        tilt : parseFloat(tilt_)
    });
    console.log(request);
    setHeadClient.callService(request);
  }

  // Handler for when the mouse is held on the up arrow.
  // Instead of writing a loop (which will block the web page), we use
  // setInterval, which repeatedly calls the given function at a given
  // time interval. In this case, it repeatedly calls move() every 50 ms.
  // Note that inside of move, we use that._timer and that.linearSpeed.
  // At the top of the file we set "var that = this" to ensure that the
  // local variable "that" always refers to this Base instance.
  this.headUp = function() {
    that._timer = setInterval(function() {
      move(that.panA, that.tiltA)
      that.tiltA -= 0.1;
      that.tiltA = Math.max(-Math.PI/4, that.tiltA);
    }, 50);
  }

  this.headDown = function() {
    that._timer = setInterval(function() {
      move(that.panA, that.tiltA)
      that.tiltA += 0.1;
      that.tiltA = Math.min(Math.PI/2, that.tiltA);
    }, 50);
  }

  this.headLeft = function() {
    that._timer = setInterval(function() {
      move(that.panA, that.tiltA)
      that.panA += 0.05;
      that.panA = Math.min(Math.PI/2, that.panA);
    }, 50);
  }

  this.headRight = function() {
    that._timer = setInterval(function() {
      move(that.panA, that.tiltA)
      that.panA -= 0.05;
      that.panA = Math.max(-Math.PI/2, that.panA);
    }, 50);
  }

  // Stops the robot from moving.
  this.stop = function() {
    if (that._timer) {
      clearInterval(that._timer);
    }
  };  

  headUp.addEventListener('mousedown', that.headUp);
  headDown.addEventListener('mousedown', that.headDown);
  headLeft.addEventListener('mousedown', that.headLeft);
  headRight.addEventListener('mousedown', that.headRight);
  
  // We bind stop() to whenever the mouse is lifted up anywhere on the webpage
  // for safety reasons. We want to be conservative about sending movement commands.
  document.addEventListener('mouseup', that.stop);
}
