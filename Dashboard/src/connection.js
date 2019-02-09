let address = document.getElementById('connect-address'),
  connect = document.getElementById('connect'),
  buttonConnect = document.getElementById('connect-button');

let loginShown = true;

// Hot Graph
class HotGraph {

  constructor(divName, pointsCount) {

    this.pointsCount = pointsCount;
    this.data = [];
    this.data.push([new Date(), 0]);

    this.graph = new Dygraph(

      // containing div
      document.getElementById(divName),
      this.data,
      { labels: ['Time', 'Value'], width: 900, height: 450 }
    );
  }

  PutValue(newValue) {
    var x = new Date();  // current time
    var y = newValue;
    this.data.push([x, y]);
    if (this.data.length > this.pointsCount) {
      this.data.splice(0, 1);  
    }

    this.graph.updateOptions({ 'file': this.data });
  }
}
// ----------

let ultrasonic_down_update = (key, variableName) => {
  ultrasonicDown.PutValue(variableName);
}
let front_encoder_update_two = (key, variableName) => {
  gearRackFrontTwo.PutValue(variableName);
}
let back_encoder_update_one = (key, variableName) => {
  gearRackBackOne.PutValue(variableName);
}
let back_encoder_update_two = (key, variableName) => {
  gearRackBackTwo.PutValue(variableName);
}
let front_encoder_update_one = (key, variableName) => {
  gearRackFrontOne.PutValue(variableName);
}
let gearRackFrontOneLimit = (key, variableName) => {
  document.getElementById('gearRackFrontOneLimit').innerHTML = variableName;
}
let gearRackFrontTwoLimit = (key, variableName) => {
  document.getElementById('gearRackFrontTwoLimit').innerHTML = variableName;
}
let gearRackBackOneLimit = (key, variableName) => {
  document.getElementById('gearRackBackOneLimit').innerHTML = variableName;
}
let gearRackBackTwoLimit = (key, variableName) => {
  document.getElementById('gearRackBackTwoLimit').innerHTML = variableName;
}
// Set function to be called when robot dis/connects
NetworkTables.addRobotConnectionListener(onRobotConnection, false);
NetworkTables.addKeyListener('/SmartDashboard/ultrasonic_front', ultrasonic_down_update);
NetworkTables.addKeyListener('/SmartDashboard/Front Gear Rack raw encoder ', front_encoder_update_one);
NetworkTables.addKeyListener('/SmartDashboard/Back Gear Rack One raw encoder ', back_encoder_update_one);
NetworkTables.addKeyListener('/SmartDashboard/Front Gear Rack Two raw encoder ', front_encoder_update_two);
NetworkTables.addKeyListener('/SmartDashboard/Back Gear Rack Two raw encoder ', back_encoder_update_two);
NetworkTables.addKeyListener('/SmartDashboard/Front Gear Rack Limit Switch', gearRackFrontOneLimit);
NetworkTables.addKeyListener('/SmartDashboard/Front Gear Rack Two Limit Switch', gearRackFrontTwoLimit);
NetworkTables.addKeyListener('/SmartDashboard/Back Gear Rack One Limit Switch', gearRackBackOneLimit);
NetworkTables.addKeyListener('/SmartDashboard/Back Gear Rack Two Limit Switch', gearRackBackTwoLimit);

let ultrasonicDown;
let gearRackFrontOne;
let gearRackFrontTwo;
let gearRackBackOne;
let gearRackBackTwo;

function OnWindowLoad() {
  ultrasonicDown = new HotGraph("ultrasonic_down", 300);
  gearRackFrontOne = new HotGraph("gear_rack_one_encoder", 300);
  gearRackFrontTwo = new HotGraph("gear_rack_two_encoder", 300);
  gearRackBackOne = new HotGraph("gear_rack_three_encoder", 300);
  gearRackBackTwo = new HotGraph("gear_rack_four_encoder", 300);
}

// Function for hiding the connect box
onkeydown = key => {
  if (key.key === 'Escape') {
    ultrasonicFront.PutValue(Math.random());
  }
};

// Called when the robot connects
function onRobotConnection(connected) {
  var state = connected ? 'Robot connected!' : 'Robot disconnected.';
  console.log(state);
  //ui.robotState.textContent = state;


  /*
  buttonConnect.onclick = () => {
    //document.body.classList.toggle('login', true);
    loginShown = true;
  };
  */

  if (connected) {
    // On connect hide the connect popup
    //document.body.classList.toggle('login', false);

    loginShown = false;
  } else if (loginShown) {
    setLogin();
  }

  if (connected) {
    document.getElementById('ConnectionHeader').innerHTML = '<h1 id="ConnectionHeader" class="uk-text-bold uk-text-success"> Hotwire Dashboard </h1>';
  } else {
    document.getElementById('ConnectionHeader').innerHTML = '<h1 id="ConnectionHeader" class="uk-text-bold uk-text-danger"> Hotwire Dashboard </h1>';
  }
}
function setLogin() {
  // Add Enter key handler
  // Enable the input and the button
  address.disabled = connect.disabled = false;
  connect.textContent = 'Connect';
  // Add the default address and select xxxx
  address.value = '10.29.90.59';
  address.focus();
  address.setSelectionRange(8, 12);
}
// On click try to connect and disable the input and the button
connect.onclick = () => {
  ipc.send('connect', address.value);
  address.disabled = connect.disabled = true;
  connect.textContent = 'Connecting...';
};
address.onkeydown = ev => {
  if (ev.key === 'Enter') {
    connect.click();
    ev.preventDefault();
    ev.stopPropagation();
  }
};

// Show login when starting
document.body.classList.toggle('login', true);
setLogin();
