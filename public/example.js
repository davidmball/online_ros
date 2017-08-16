
var example_info;

// ROS code
var ros = new ROSLIB.Ros();
var roslib_connected = false;
var interval_active = false;
var interval;

var interval_get_ros_details;

var viewer;
var markerClient;

function try_ros_connection() {
  if (roslib_connected == false)
  {
    ros.connect('ws://' + document.getElementById("host_name").value + ':' + document.getElementById("rosbridge_port").value);
  }
}

ros.on('connection', function() {
  roslib_connected = true;
  console.log('Connected to websocket server.');
  interval_active = false;
  clearInterval(interval);
  interval_get_ros_details = setInterval(get_ros_details, 500);

  for (var id = 0; id < 2; id++)
  {
    // set the defaults from the xml file
    if (example_info.example.feedback[id].tab == "topics" &&
      typeof example_info.example.feedback[id].topic[0] != "undefined")
    {
      set_topic(example_info.example.feedback[id].topic[0], id);
    }
    else if (example_info.example.feedback[id].tab == "viz")
    {
      if (typeof viewer != "undefined") {
        viewer.unsubscribe();
        delete viewer;
      }

      // TODO: The width and height need to come from somewhere meaningful
      viewer = new ROS3D.Viewer({
        divID : 'roslib_viewer' + id,
        width : 300,
        height : 200,
        antialias : true
      });

      if (typeof markerClient != "undefined") {
        markerClient.unsubscribe();
        delete markerClient;
      }
      markerClient = new ROS3D.MarkerClient({
        ros : ros,
        tfClient : new ROSLIB.TFClient({
          ros : ros,
          angularThres : 0.01,
          transThres : 0.01,
          rate : 10.0,
          fixedFrame : example_info.example.feedback[id].frame[0]
        }),
        topic : example_info.example.feedback[id].topic[0],
        rootObject : viewer.scene
      });
    }
  }

});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  clearInterval(interval_get_ros_details);
  console.log('Connection to websocket server closed.');
});

var selected_topic_name;
var selected_param_name;
var topic_listener;

function set_topic(topic_name, id) {
  selected_topic_name = topic_name;
  var topic_type;
  if (topic_name == '/rosout' || topic_name == 'rosout_agg') {
    topic_type = 'rosgraph_msgs/Log';
  } else if (topic_name == example_info.example.feedback[id].topic[0]) {
    topic_type = example_info.example.feedback[id].msg_type[0];
  } else {
    // hope this works.
    topic_type = 'std_msgs/String';
  }

  if (typeof topic_listener != "undefined") {
    topic_listener.unsubscribe();
    delete topic_listener;
    document.getElementById("show_topic" + id).value = '';
  }

    // TODO: Why do I need to set the messagetype for the listener?
  topic_listener = new ROSLIB.Topic({
    ros : ros,
    name : topic_name,
    messageType : topic_type
  });

  topic_listener.subscribe(function(message) {
    // set 2 space tabs and get rid of the ""
    console.log("topic_listener.subscribe" + id)
    message = JSON.stringify(message, null, 2).replace(/\"([^(\")"]+)\":/g,"$1:");
    document.getElementById("show_topic" + id).value += message + '\r';
    document.getElementById("show_topic" + id).scrollTop = document.getElementById("show_topic" + id).scrollHeight;
  });
}

function topic_list_select(element) {
  set_topic(element.value, element.id.slice(-1));
}

var roslib_param;
function set_param(param) {
  selected_param_name = param;
  if (typeof roslib_param != "undefined") {
    delete roslib_param;
  }
  roslib_param = new ROSLIB.Param({
    ros: ros,
    name: selected_param_name
  });
}

function param_list_select() {
  var sel = document.getElementById("param_list");
  var sel_val = sel.options[sel.selectedIndex].value;
  set_param(sel_val);
  roslib_param.get(function(value) {
    document.getElementById("show_param").value = value;
  });
}

/**
 * Gets and updates the list of ROS topics, nodes, services, and parameters.
 * This is repeatidly called at an interval whilst the ROS bridge is connected.
 * TODO: It would be much better if this was pushed to us when a change happens.
 * At the least we could store the callback values and check if they have changed.
 */
function get_ros_details() {
  ros.getTopics(function(topics) {
    // TODO: It's a shame this doesn't also return the message types.
    for (var id = 0; id < 2; id++)
    {
      var sel = document.getElementById('topic_list' + id);
      sel.options.length = 0;
      for (var topic in topics)
      {
        // TODO: Also work out a way to remove the tf's that appear ... but only if they shouldn't!
        // This might need to become a user configurable parameter.
        if (topics[topic].indexOf('tf2_web_republisher') == -1)
        {
          var opt = document.createElement('option');
          opt.innerHTML = topics[topic];
          opt.value = topics[topic];
          sel.appendChild(opt);
        }
      }
      sel.value = selected_topic_name;
    }
  });
  ros.getNodes(function(nodes) {
    for (var id = 0; id < 2; id++)
    {
      var sel = document.getElementById('node_list' + id);
      sel.options.length = 0;
      for (var node in nodes)
      {
        if (nodes[node].indexOf('tf2_web_republisher') == -1 &&
            nodes[node].indexOf('rosbridge_websocket') == -1 )
        {
          var opt = document.createElement('option');
          opt.innerHTML = nodes[node];
          opt.value = nodes[node];
          sel.appendChild(opt);
        }
      }
    }
  });
  ros.getServices(function(services) {
    for (var id = 0; id < 2; id++)
    {
      var sel = document.getElementById('service_list' + id);
      sel.options.length = 0;
      for (var service in services)
      {
        var opt = document.createElement('option');
        opt.innerHTML = services[service];
        opt.value = services[service];
        sel.appendChild(opt);
      }
   }
  });
  ros.getParams(function(params) {
    for (var id = 0; id < 2; id++)
    {
      var sel = document.getElementById('param_list' + id);
      sel.options.length = 0;
      for (var param in params)
      {
        if (params[param].indexOf('rosbridge_websocket') == -1 &&
            params[param].indexOf('rosapi') == -1)
        {
          var opt = document.createElement('option');
          opt.innerHTML = params[param];
          opt.value = params[param];
          sel.appendChild(opt);
        }
      }
      if (typeof selected_param_name != "undefined")
      {
        sel.value = selected_param_name;
      }
    }
  });
}

/**
 * This function updates the terminal feedback textbox.
 */
$(document).ready(function(){
  var socket = io();
  socket.on('compile_output',function(msg){
    for (var id = 0; id < 2; id++)
    {
      document.getElementById("show_terminal" + id).value += msg;
      document.getElementById("show_terminal" + id).scrollTop = document.getElementById("show_terminal" + id).scrollHeight
    }
  });
});



var files = {};
var editor = ace.edit("editor");
editor.setTheme("ace/theme/chrome");
var EditSession = ace.require("ace/edit_session").EditSession;
var editor_sessions = {};

var example_name = location.search.split('name=')[1];

$.get("/get_files?name=" + example_name, function (data) {

  // parse the data into the files for the editor and the general example information
   var parsed_data = JSON.parse(data);
   files = parsed_data[0];
   example_info = parsed_data[1];

   // load each file into its own editor
   var sel = document.getElementById('file_list');
   for (var filename in files)
   {
     var opt = document.createElement('option');
     opt.value = filename;
     opt.innerHTML = filename;
     sel.appendChild(opt);
     var filename_extension = filename.split('.').pop();
     editor_sessions[filename] = new EditSession(filename);
     if (filename_extension == 'cpp' || filename_extension == 'c')
          editor_sessions[filename].setMode("ace/mode/c_cpp");
     else if (filename_extension == 'xml')
          editor_sessions[filename].setMode("ace/mode/xml");
     else
          editor_sessions[filename].setMode("ace/mode/text");
     editor_sessions[filename].setValue(files[filename], -1);
   }

   // set the initial file for editing
   // TODO: Test if this is set / valid
   editor.setSession(editor_sessions[example_info.example.start_file]);
   sel.value = example_info.example.start_file;

   // set other parts of the page
   document.getElementById("run_cmd").innerHTML = example_info.example.run_cmd;
   document.getElementById("example_title").innerHTML = example_info.example.title;
   document.getElementById("description").innerHTML = example_info.example.description;
   for (var id = 0; id < 2; id++)
   {
      document.getElementById("tab_" + example_info.example.feedback[id].tab + id).checked = true;
   }
});


function file_list_select() {
  var sel = document.getElementById("file_list");
  var sel_val = sel.options[sel.selectedIndex].value;
  editor.setSession(editor_sessions[sel_val]);
  if (sel_val == 'CMakeLists.txt' || sel_val == 'package.xml')
  {
    editor.setReadOnly(true);
  } else {
    editor.setReadOnly(false);
  }
}

var runningCode = false;

$('#compile').on('click', function() {
  if (runningCode)
    return;
  runningCode = true;

  document.getElementById("compile").style.background='#FF8C00';
  roslib_connected = false;
  if (!interval_active)
  {
     interval_active = true;
     interval = setInterval(try_ros_connection, 2000);
  }

  for (var id = 0; id < 2; id++)
  {
    document.getElementById("show_terminal" + id).value = '';
    document.getElementById("show_topic" + id).value = '';
  }

  var code = {};
  for (var filename in editor_sessions)
  {
    code[filename] = editor_sessions[filename].getValue();
  }

  var json = {
    code_files: code,
    rosbridge_port: document.getElementById("rosbridge_port").value
  };

  $.post("/compile?name=" + example_name, json, function(data, error, xhr) {
      document.getElementById("compile").style.background='#00FF00';
      runningCode = false;
  });
});
