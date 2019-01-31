let address = document.getElementById('connect-address'),
  connect = document.getElementById('connect'),
  buttonConnect = document.getElementById('connect-button');

let loginShown = true;

// Set function to be called on NetworkTables connect. Not implemented.
//NetworkTables.addWsConnectionListener(onNetworkTablesConnection, true);

// Set function to be called when robot dis/connects
NetworkTables.addRobotConnectionListener(onRobotConnection, false);

// Sets function to be called when any NetworkTables key/value changes
//NetworkTables.addGlobalListener(onValueChanged, true);

function OnWindowLoad () 
{
		
	var data = [];
    var t = new Date();
    for (var i = 10; i >= 0; i--) {
      var x = new Date(t.getTime() - i * 1000);
      data.push([x, Math.random()]);
    }

 new Dygraph(

    // containing div
    document.getElementById("graph_ultrasonic"),

    // CSV or path to a CSV file.
    "Date,Temperature\n" +
    "2008-05-07,75\n" +
    "2008-05-08,70\n" +
    "2008-05-09,80\n",
	{ width:1300, height:500 }
	);
    
	
	
	/*
		 new Dygraph(
            document.getElementById("div_g"),
				"Date,Temperature\n" +
				"2008-05-07,75\n" +
				"2008-05-08,70\n" +
				"2008-05-09,80\n",
			{
              rollPeriod: 7,
              errorBars: true
            }
          );
		  
		  
    var g = new Dygraph(document.getElementById("graph_ultrasonic"), data,
                        {
                          drawPoints: true,
                          showRoller: true,
                          valueRange: [0.0, 1.2],
                          labels: ['Time', 'Random']
                        });
    // It sucks that these things aren't objects, and we need to store state in window.
    window.intervalId = setInterval(function() {
      var x = new Date();  // current time
      var y = Math.random();
      data.push([x, y]);
      g.updateOptions( { 'file': data } );
    }, 1000);  
	*/
}
	
// Function for hiding the connect box
onkeydown = key => {
  if (key.key === 'Escape') {
    document.body.classList.toggle('login', false);
    loginShown = false;
  }
};

/**
 * Function to be called when robot c
 onnects
 * @param {boolean} connected
 */
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
