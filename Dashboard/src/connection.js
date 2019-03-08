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

let ultrasonicDown;
let gearRackFrontOne;
let gearRackFrontTwo;

let ultrasonic_down_update = (key, variableName) => {
  ultrasonicDown.PutValue(variableName);
}
let front_encoder_update_two = (key, variableName) => {
  gearRackFrontTwo.PutValue(variableName);
}
let front_encoder_update_one = (key, variableName) => {
  gearRackFrontOne.PutValue(variableName);
}
let gear_rack_front_one_limit_update = (key, variableName) => {
  document.getElementById('gear_rack_front_one_limit').innerHTML = variableName;
}
let gear_rack_front_two_limit_update = (key, variableName) => {
  document.getElementById('gear_rack_front_two_limit').innerHTML = variableName;
}
let gear_rack_back_one_limit_update = (key, variableName) => {
  document.getElementById('gear_rack_back_one_limit').innerHTML = variableName;
}
let gear_rack_back_two_limit_update = (key, variableName) => {
  document.getElementById('gear_rack_back_two_limit').innerHTML = variableName;
}

NetworkTables.addRobotConnectionListener(onRobotConnection, false);
NetworkTables.addKeyListener('/SmartDashboard/UltrasonicDown', ultrasonic_down_update);
NetworkTables.addKeyListener('/SmartDashboard/FGR1 raw encoder ', front_encoder_update_one);
NetworkTables.addKeyListener('/SmartDashboard/FGR2 raw encoder ', front_encoder_update_two);
NetworkTables.addKeyListener('/SmartDashboard/FGR1 Limit', gear_rack_front_one_limit_update);
NetworkTables.addKeyListener('/SmartDashboard/BGR1 Limit', gear_rack_back_one_limit_update);
NetworkTables.addKeyListener('/SmartDashboard/FGR2 Limit', gear_rack_front_two_limit_update);
NetworkTables.addKeyListener('/SmartDashboard/BGR2 Limit', gear_rack_back_two_limit_update);

function OnWindowLoad() {
  ultrasonicDown = new HotGraph("ultrasonic_down", 300);
  gearRackFrontOne = new HotGraph("gear_rack_one_encoder", 300);
  gearRackFrontTwo = new HotGraph("gear_rack_two_encoder", 300);
}


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
  address.value = '10.29.90.2';
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
