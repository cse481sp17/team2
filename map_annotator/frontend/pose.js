Pose = function(ros, name, parent) {
  var that = this;
  this.name = name;
  this.parent = parent;

  function handleGoTo() {
    console.log('Go to ' + name + ' clicked.');
    parent.handleAction("goto", name);
  }

  function handleDelete() {
    console.log('Delete ' + name + ' clicked.');
    parent.handleAction("delete", name);
  }

  this.render = function() {
    var node = document.createElement('div');
    var nameNode = document.createTextNode(name);
    node.appendChild(nameNode)

    var sendNode = document.createElement('input');
    sendNode.type = 'button';
    sendNode.value = 'Go to';
    sendNode.addEventListener('click', handleGoTo);
    node.appendChild(sendNode);

    var deleteNode = document.createElement('input');
    deleteNode.type = 'button';
    deleteNode.value = 'Delete';
    deleteNode.addEventListener('click', handleDelete);
    node.appendChild(deleteNode);
    return node;
  }
}
