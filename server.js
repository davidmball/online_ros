const http = require('http');
const fs = require('fs');
const express = require('express');
const path = require('path')
const io   = require('socket.io')(http);
const dir = require('node-dir');
const bodyParser = require('body-parser');
const crypto = require("crypto");

var app = express();
app.use(express.static('public'));
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({extended: true}));

var example_name = 'chatter';
var files = {};
var example_path = '/home/david/ros_examples/src/topics/';



dir.readFiles(example_path,
    function(err, content, filename, next) {
        if (err)
            throw err;
        files[filename.split("/").pop()] = content;
  //      console.log('filename:', filename);
    //    console.log('content:', content);
        next();
    },
    function(err, files){
        if (err)
            throw err;
        console.log('finished reading files:', files);
    });

io.on('connection', function(socket){
  console.log('a user connected');
});


app.get('/chatter.html', function (req, res) {
   fs.readFile( __dirname + "/" + "chatter.html", function(err, data) {
        if (err) {
            res.send(404);
        } else {
            res.contentType('text/html');
            data = data.toString().replace(/\{\{example_title\}\}/, example_name);
            res.send(data);
        }
    });
});

app.get("/get_files", function (req, res) {
  res.send(JSON.stringify(files));
});

var server = app.listen(8080, function () {
   var host = server.address().address;
   var port = server.address().port;

   console.log("Example app listening at http://%s:%s", host, port);
})

var socket = io.listen(server);


app.post('/compile', function(req, res)
{
    console.log("Writing code.");
    fs.writeFileSync('/home/david/catkin_ws/src/topics/CMakeLists.txt', files['CMakeLists.txt']);
    fs.writeFileSync('/home/david/catkin_ws/src/topics/package.xml', files['package.xml']);
    fs.writeFileSync('/home/david/catkin_ws/src/topics/src/chatter.cpp', req.body.code_files['chatter.cpp']);

    console.log("Compiling code.");
    var spawn = require('child_process').spawn;
    var command = './run_docker.sh'
    var docker_id = crypto.randomBytes(16).toString("hex");
    var net_id = crypto.randomBytes(16).toString("hex");
    var args = ['10', docker_id, 'ros_workspace_rosbridge', '/home/david/catkin_ws/src', net_id];
    var ls = spawn(command, args);

    ls.stdout.on('data', (data) => {
      //console.log(`stdout: ${data}`);
      // TODO: Actually it would be great if the text was rendered in colour in the html page
      var data_string = data.toString().replace(/[\u001b\u009b][[()#;?]*(?:[0-9]{1,4}(?:;[0-9]{0,4})*)?[0-9A-ORZcf-nqry=><]/g, '');
      io.emit('compile_output', data_string);
    });

    ls.stderr.on('data', (data) => {
      console.log(`stderr: ${data}`);
    });

    ls.on('close', (code) => {
      console.log(`child process exited with code ${code}`);
      res.end('true');
    });
});
