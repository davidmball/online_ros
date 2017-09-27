/*
 * Copyright (c) 2017, David Ball
 * All rights reserved.
 *
 * 3-clause BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


var exampleInfo

// ROS code
var ros = new ROSLIB.Ros()
var connectedROSLIB = false
var intervalActive = false
var intervalROS
var intervalGetROSDetails

var viewer
var markerClient

var selectedTopicName = ['undefined', 'undefined']
var selectedParamName = ['undefined', 'undefined']
var topicListener = []
var codeLanguage = ""

function tryROSConnection () {
  if (connectedROSLIB === false) {
    ros.connect('ws://' + document.getElementById('host_name').value + ':' + document.getElementById('rosbridge_port').value)
  //  ros.connect('ws://localhost:9090')
  }
}

ros.on('connection', function () {
  connectedROSLIB = true
  console.log('Connected to websocket server.')
  intervalActive = false
  clearInterval(intervalROS)
  intervalGetROSDetails = setInterval(getROSDetails, 1000)

  for (var id = 0; id < 2; id++) {
    // set the defaults from the xml file
    if (exampleInfo.example.feedback[id].tab[0] === 'topics') {
      selectedTopicName[id] = exampleInfo.example.feedback[id].topic[0]
    } else if (exampleInfo.example.feedback[id].tab[0] === 'viz') {
      if (typeof markerClient !== 'undefined') {
        markerClient = null
      }

      if (typeof viewer === 'undefined') {
        // TODO: The width and height need to come from somewhere meaningful
        viewer = new ROS3D.Viewer({
          divID: 'roslib_viewer' + id,
          width: 300,
          height: 200,
          antialias: true
        })
      }

      markerClient = new ROS3D.MarkerClient({
        ros: ros,
        tfClient: new ROSLIB.TFClient({
          ros: ros,
          angularThres: 0.01,
          transThres: 0.01,
          rate: 10.0,
          fixedFrame: exampleInfo.example.feedback[id].frame[0]
        }),
        topic: exampleInfo.example.feedback[id].topic[0],
        rootObject: viewer.scene
      })
    }
  }
})

ros.on('error', function (error) {
  console.log('Error connecting to websocket server: ', error)
})

ros.on('close', function () {
  clearInterval(intervalGetROSDetails)
  for (var id = 0; id < 2; id++) {
     topicListener[id] = null
  }
  console.log('Connection to websocket server closed.')
})

function setTopicSub (topicName, id) {
  if (topicName === selectedTopicName[id] && typeof topicListener[id] !== 'undefined' && topicListener[id] !== null) {
    return
  } else {
    selectedTopicName[id] = topicName
  }

  var topicType
  if (topicName === '/rosout' || topicName === 'rosout_agg') {
    topicType = 'rosgraph_msgs/Log'
  } else if (typeof exampleInfo.example.feedback[id].topic !== 'undefined' && topicName === exampleInfo.example.feedback[id].topic[0]) {
    topicType = exampleInfo.example.feedback[id].msg_type[0]
  } else {
    // hope this works.
    topicType = 'std_msgs/String'
  }

  if (typeof topicListener[id] !== 'undefined' && topicListener[id] !== null) {
    topicListener[id].unsubscribe()
    topicListener[id] = null
    document.getElementById('show_topic' + id).value = ''
  }

    // TODO: Why do I need to set the messagetype for the listener?
  topicListener[id] = new ROSLIB.Topic({
    ros: ros,
    name: topicName,
    messageType: topicType
  })

  topicListener[id].subscribe(function (message) {
    // set 2 space tabs and get rid of the ""
    // message = JSON.stringify(message, null, 2).replace(/\"([^(\")"]+)\":/g, '$1:')
    message = JSON.stringify(message, null, 2).replace(/([^()"]+):/g, '$1:')
    document.getElementById('show_topic' + id).value += message + '\r'
    document.getElementById('show_topic' + id).scrollTop = document.getElementById('show_topic' + id).scrollHeight
  })
}

function onTopicListSelect (element) { // eslint-disable-line
  setTopicSub(element.value, element.id.slice(-1))
}

var paramROSLIB
function setParam (param) {
  selectedParamName = param
  if (typeof paramROSLIB !== 'undefined') {
    paramROSLIB = null
  }
  paramROSLIB = new ROSLIB.Param({
    ros: ros,
    name: selectedParamName
  })
}

function onParamListSelect (element) { // eslint-disable-line
  var selValue = element.value
  setParam(selValue)
  paramROSLIB.get(function (value) {
    document.getElementById('show_param').value = value
  })
}

/**
 * Gets and updates the list of ROS topics, nodes, services, and parameters.
 * This is repeatidly called at an interval whilst the ROS bridge is connected.
 * TODO: It would be much better if this was pushed to us when a change happens.
 * At the least we could store the callback values and check if they have changed.
 */
function getROSDetails () {

  ros.getTopics(function (topics) {
    // TODO: It's a shame this doesn't also return the message types.
    for (var id = 0; id < 2; id++) {
      var sel = document.getElementById('topic_list' + id)
      sel.options.length = 0
      for (var topic in topics) {
        // TODO: Also work out a way to remove the tf's that appear ... but only if they shouldn't!
        // This might need to become a user configurable parameter.
        if (topics[topic].indexOf('tf2_web_republisher') === -1) {
          var opt = document.createElement('option')
          opt.innerHTML = topics[topic]
          opt.value = topics[topic]
          sel.appendChild(opt)
        }
        if (selectedTopicName[id] === topics[topic])
          setTopicSub(selectedTopicName[id], id)
      }
      sel.value = selectedTopicName[id]
    }
  })
  ros.getNodes(function (nodes) {
    for (var id = 0; id < 2; id++) {
      var sel = document.getElementById('node_list' + id)
      sel.options.length = 0
      for (var node in nodes) {
        if (nodes[node].indexOf('tf2_web_republisher') === -1 &&
            nodes[node].indexOf('rosbridge_websocket') === -1) {
          var opt = document.createElement('option')
          opt.innerHTML = nodes[node]
          opt.value = nodes[node]
          sel.appendChild(opt)
        }
      }
    }
  })
  ros.getServices(function (services) {
    for (var id = 0; id < 2; id++) {
      var sel = document.getElementById('service_list' + id)
      sel.options.length = 0
      for (var service in services) {
        var opt = document.createElement('option')
        opt.innerHTML = services[service]
        opt.value = services[service]
        sel.appendChild(opt)
      }
    }
  })
  ros.getParams(function (params) {
    for (var id = 0; id < 2; id++) {
      var sel = document.getElementById('param_list' + id)
      sel.options.length = 0
      for (var param in params) {
        if (params[param].indexOf('rosbridge_websocket') === -1 &&
            params[param].indexOf('rosapi') === -1) {
          var opt = document.createElement('option')
          opt.innerHTML = params[param]
          opt.value = params[param]
          sel.appendChild(opt)
        }
      }
      if (typeof selectedParamName[id] !== 'undefined') {
        sel.value = selectedParamName[id]
      }
    }
  })
}

/**
 * This function updates the terminal feedback textbox.
 */
$(document).ready(function () {
  var socket = io()
  var port = document.getElementById('rosbridge_port').value
  socket.on('compile_output' + port, function (msg) {
    if (msg.indexOf('/rosbridge_websocket') >=0)
    {
      connectedROSLIB = false
      if (!intervalActive) {
        intervalActive = true
        intervalROS = setInterval(tryROSConnection, 1000)
      }
    } else {

      for (var id = 0; id < 2; id++) {
        document.getElementById('show_terminal' + id).value += msg
        document.getElementById('show_terminal' + id).scrollTop = document.getElementById('show_terminal' + id).scrollHeight
      }
    }
  })
})

var files = {}
var editor = {}
var editorDiv = {}
var yjs_rooms = {}

var exampleName = location.search.split('name=')[1]
var share = location.search.split('share=')[1]
var shareString;
if (typeof share == 'undefined') {
  var s = "abcdefghijklmnopqrstuvwxyz0123456789";
  shareString = Array(6).join().split(',').map(function() { return s.charAt(Math.floor(Math.random() * s.length)); }).join('');
} else {
  shareString = share
}

if (document.getElementById('host_name').value == 'localhost') {
  document.getElementById('share_link').value = document.getElementById('host_name').value + ':8080' + '/example.html?name=' + exampleName + '&share=' + shareString
} else {
  document.getElementById('share_link').value = document.getElementById('host_name').value + '/example.html?name=' + exampleName + '&share=' + shareString
}
$.get('/get_files?name=' + exampleName, function (data) {

  // parse the data into the files for the editor and the general example information
  var parsedData = JSON.parse(data)
  files = parsedData[0]
  exampleInfo = parsedData[1]

   // load each file into its own editor
  var num = 0
  for (var filename in files) {
    var filenameExtension = filename.split('.').pop()
    var formattedNumber = ("0" + num).slice(-2)
    editor[filename] = ace.edit('editor'+formattedNumber)
    editor[filename].setTheme('ace/theme/chrome')
    editor[filename].setPrintMarginColumn(120)
    editor[filename].$blockScrolling = 1
    editorDiv[filename] = document.getElementById('editor'+formattedNumber)
    num += 1

    if (filenameExtension === 'cpp' || filenameExtension === 'c')
    { editor[filename].session.setMode('ace/mode/c_cpp') }
    else if (filenameExtension === 'xml')
    { editor[filename].session.setMode('ace/mode/xml') }
    else if (filenameExtension === 'py')
    { editor[filename].session.setMode('ace/mode/python')}
    else
    { editor[filename].session.setMode('ace/mode/text') }



    if (filename === '/CMakeLists.txt' || filename === '/package.xml') {
      editor[filename].setReadOnly(true)
      editor[filename].session.setValue(files[filename], -1)
    } else {
      editor[filename].setReadOnly(false)

      yjs_rooms[filename] = new Promise((resolve) => {
        Y({
          db: {
            name: 'memory',
            namespace: 'ace-example' + filename
          },
          connector: {
            name: 'websockets-client',
            room: shareString + filename,
            url: 'http://' + document.getElementById('host_name').value + ':1234',
            filename: filename
          },
          sourceDir: '/bower_components',
          share: {
            ace: 'Text' // y.share.textarea is of type Y.Text
          }
        }).then(function (y) {
         y.share.ace.bindAce(editor[y.options.connector.filename])
         if (typeof share === 'undefined')
           editor[y.options.connector.filename].session.setValue(files[y.options.connector.filename], -1)
         resolve(y)
        });
      });
    }
  } // for filename in files
  editorDiv[exampleInfo.example.language[0].start_file].style.display = 'block'

  // Make a list of the languages
  var sel = document.getElementById('language_list')
  sel.options.length = 0
  for (var lang in exampleInfo.example.language) {
    var opt = document.createElement('option')
    opt.innerHTML = exampleInfo.example.language[lang].name
    opt.value = exampleInfo.example.language[lang].name
    sel.appendChild(opt)
  }
  sel.value = exampleInfo.example.language[0].name
  codeLanguage = exampleInfo.example.language[0].name[0]
  setFileList(codeLanguage, 0)

   // set other parts of the page
  document.getElementById('run_cmd').innerHTML = exampleInfo.example.language[0].run_cmd
  document.getElementById('example_title').innerHTML = exampleInfo.example.title
  document.getElementById('description').innerHTML = exampleInfo.example.description
  for (var id = 0; id < 2; id++) {
    document.getElementById('tab_' + exampleInfo.example.feedback[id].tab + id).checked = true
  }

  document.getElementById('run_timer').innerHTML = "Run Time: " + exampleInfo.example.time_limit + " seconds"
})

function onFileListSelect (element) { // eslint-disable-line
  var selValue = element.value
  for (filename in editorDiv) {
    editorDiv[filename].style.display = 'none'
  }
  editorDiv[selValue].style.display = 'block'
}

function setFileList(language, langID) {
  var sel = document.getElementById('file_list')
  sel.options.length = 0;
  for (var filename in files) {
    var filenameExtension = filename.split('.').pop()
    if (language === 'cpp' && filenameExtension == 'py')
      continue;
    if (language === 'py' && (filenameExtension == 'cpp' || filenameExtension == 'c' || filenameExtension == 'h' || filenameExtension == 'hpp'))
      continue;
    var opt = document.createElement('option')
    opt.value = filename
    opt.innerHTML = filename
    sel.appendChild(opt)
  }
  sel.value = exampleInfo.example.language[langID].start_file
}

function onLanguageSelect (element) { // eslint-disable-line
  codeLanguage = element.value
  for (var lang in exampleInfo.example.language)
  {
    if (codeLanguage === exampleInfo.example.language[lang].name[0]) {
      document.getElementById('run_cmd').innerHTML = exampleInfo.example.language[lang].run_cmd
      for (filename in editorDiv) {
        editorDiv[filename].style.display = 'none'
      }
      editorDiv[exampleInfo.example.language[lang].start_file].style.display = 'block'
      setFileList(codeLanguage, lang)
    }
  }
}

var runningCode = false

$('#compile').on('click', function () {
  if (runningCode) { return }
  runningCode = true

  document.getElementById('compile').style.background = '#FF8C00'

  for (var id = 0; id < 2; id++) {
    document.getElementById('show_terminal' + id).value = ''
    document.getElementById('show_topic' + id).value = ''
    var sel = document.getElementById('topic_list' + id)
    sel.options.length = 0
    var sel = document.getElementById('node_list' + id)
    sel.options.length = 0
    var sel = document.getElementById('param_list' + id)
    sel.options.length = 0
    var sel = document.getElementById('service_list' + id)
    sel.options.length = 0
  }
  selectedTopicName = ['undefined', 'undefined']
  selectedParamName = ['undefined', 'undefined']


  var code = {}
  for (var filename in editor) {
    code[filename] = editor[filename].session.getValue()
  }

  var json = {
    code_files: code,
    rosbridge_port: document.getElementById('rosbridge_port').value,
    language : codeLanguage
  }

  var time_remaining = exampleInfo.example.time_limit
  var run_countdown = setInterval(function() {
    document.getElementById('run_timer').innerHTML = "Run Time: " + time_remaining + " seconds"
    time_remaining = time_remaining - 1.0
    if (time_remaining === 1)
    {
      connectedROSLIB = false
      console.log('Closing connecting to ROS bridge.')
      intervalActive = false
      clearInterval(intervalROS)
    }
  }, 1000);

  $.post('/compile?name=' + exampleName, json, function (data, error, xhr) {
    clearInterval(run_countdown)
    document.getElementById('compile').style.background = '#00FF00'
    runningCode = false
    document.getElementById('run_timer').innerHTML = "Run Time: " + exampleInfo.example.time_limit + " seconds"
  })
})
