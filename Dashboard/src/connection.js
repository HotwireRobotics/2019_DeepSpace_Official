let address = document.getElementById('connect-address');
let connect = document.getElementById('connect');
let buttonConnect = document.getElementById('connect-button');

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
let gearRackCenter;
let potGraph;
let navxGraph;

let navx = (key, variableName) => {
  navxGraph.PutValue(variableName);
}
let pot = (key, variableName) => {
  potGraph.PutValue(variableName);
}
let ultrasonic_down_update = (key, variableName) => {
  ultrasonicDown.PutValue(variableName);
}
let front_encoder_update_two = (key, variableName) => {
  gearRackFrontTwo.PutValue(variableName);
}
let front_encoder_update_one = (key, variableName) => {
  gearRackCenter.PutValue(variableName);
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
let update_match_time = (key, variableName) => {
  document.getElementById('header').innerHTML = 'Hotwire Dashboard ' + variableName;
}
let update_auto_choice_display = (key, variableName) => {
  document.getElementById('AutoChoiceDisplay').innerHTML = variableName;
}

NetworkTables.addRobotConnectionListener(onRobotConnection, false);
NetworkTables.addKeyListener('/SmartDashboard/UltrasonicDown', ultrasonic_down_update);
NetworkTables.addKeyListener('/SmartDashboard/FGR1 raw encoder ', front_encoder_update_one);
NetworkTables.addKeyListener('/SmartDashboard/FGR1 Limit', gear_rack_front_one_limit_update);
NetworkTables.addKeyListener('/SmartDashboard/BGR1 Limit', gear_rack_back_one_limit_update);
NetworkTables.addKeyListener('/SmartDashboard/BGR2 Limit', gear_rack_back_two_limit_update);
NetworkTables.addKeyListener('/SmartDashboard/Navx Value', navx);
NetworkTables.addKeyListener('/SmartDashboard/Pot Value', pot);
NetworkTables.addKeyListener('/FMSInfo/MatchTime', update_match_time);
NetworkTables.addKeyListener('/SmartDashboard/autoSelect', update_auto_choice_display);

function OnWindowLoad() {
  ultrasonicDown = new HotGraph("ultrasonic_down", 300);
  gearRackCenter = new HotGraph("gear_rack_center_encoder", 300);
  potGraph = new HotGraph("pot", 300);
  navxGraph = new HotGraph("navx", 300);
}

// Constantly set autonomous
setTimeout(setAutonomous,  100);

// Called when the robot connects
function onRobotConnection(connected) {
  var state = connected ? 'Robot connected!' : 'Robot disconnected.';
  console.log(state);


  if (connected) {
    loginShown = false;
  } else if (loginShown) {
    setLogin();
  }

  if (connected) {
    document.getElementById('ConnectionHeader').innerHTML = '<h1 id="header" class="uk-text-bold uk-text-success"> Hotwire Dashboard </h1>';
  } else {
    document.getElementById('ConnectionHeader').innerHTML = '<h1 id="header" class="uk-text-bold uk-text-danger"> Hotwire Dashboard </h1>';
  }
}

// Connect to the robot
function Connect(address) {
  document.getElementById('ConnectionHeader').innerHTML = '<h1 id="header" class="uk-text-bold uk-text-warning"> Hotwire Dashboard </h1>';
  ipc.send('connect', address);
}

function setAutonomous() {
  NetworkTables.putValue("/SmartDashboard/autoSelect", document.getElementById('autoSelect').value);
}

function setLogin() {
  // Add Enter key handler
  // Enable the input and the button
  address.disabled = connect.disabled = false;
  connect.textContent = 'Connect';
  // Add the default address and select xxxx
  address.value = '10.29.90.2';
  address.setSelectionRange(8, 12);
}
// On click try to connect and disable the input and the button
connect.onclick = () => {
  Connect(address.value);
  ipc.send('connect', address.value);
};
// address.onkeydown = ev => {
//   if (ev.key === 'Enter') {
//     connect.click();
//     ev.preventDefault();
//     ev.stopPropagation();
//   }
// };

// Show login when starting
document.body.classList.toggle('login', true);
setLogin();

function ConnectField() {
  Connect('10.29.90.2');
}

function ConnectWired() {
  Connect('172.22.11.2');
}