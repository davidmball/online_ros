const http = require('http');
const fs = require('fs-extra');
const express = require('express');
const path = require('path')
const io   = require('socket.io')(http);
const dir = require('node-dir');
const bodyParser = require('body-parser');
const crypto = require("crypto");
const config = require('config');

var app = express();
app.use(express.static('public'));
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({extended: true}));

var example_path = config.get('example_path');
var compile_path = config.get('compile_path');
var host_port = config.get('host_port');
var host = config.get('host');
var rosbridge_port = config.get('rosbridge_port');


var files = [];
dir.readFiles(example_path,
    function(err, content, filename, next) {
        if (err)
            throw err;

        var filename2 = filename;
        var example_name = filename.replace(example_path, '').split("/")[0];
        files.push([example_name, filename2.replace(example_path + example_name, ''), content]);
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


app.get('/example.html', function (req, res) {
   if (typeof req.query.name == 'undefined' || !req.query.name)
   {
     res.redirect('index.html');
   } else {

     var example_name = req.query.name;

     fs.readFile( __dirname + "/" + "example.html", function(err, data) {
          if (err) {
              res.send(404);
          } else {
              res.contentType('text/html');
              data = data.toString().replace(/\{\{example_title\}\}/g, example_name);
              data = data.toString().replace(/\{\{example_name\}\}/g, example_name);
              data = data.toString().replace(/\{\{host_name\}\}/g, host);
              data = data.toString().replace(/\{\{rosbridge_port\}\}/g, rosbridge_port);

              res.send(data);
          }
      });
    }
});

app.get("/get_files", function (req, res) {
  var example_name = req.query.name;
  var example_files = {};
  for (var i = 0; i < files.length; i++)
  {
    if (files[i][0] == example_name)
      example_files[files[i][1]] = files[i][2];
  }
  res.send(JSON.stringify(example_files));
});

var server = app.listen(host_port, function () {
   var host = server.address().address;
   var port = server.address().port;

   console.log("Example app listening at http://%s:%s", host, port);
})

var socket = io.listen(server);



app.post('/compile', function(req, res)
{
    var temp_dir = crypto.randomBytes(16).toString("hex");

    var example_name = req.query.name;

    console.log("Writing code.");
    for (var filename in req.body.code_files)
    {
      if (filename == '/CMakeLists.txt' || filename == '/package.xml')
        continue;
      fs.outputFileSync(compile_path + temp_dir + '/' + example_name + '/' + filename, req.body.code_files[filename]);
    }
    var example_files = {};
    for (var i = 0; i < files.length; i++)
    {
      if (files[i][0] == example_name)
        example_files[files[i][1]] = files[i][2];
    }
    fs.outputFileSync(compile_path + temp_dir + '/' + example_name + '/' + 'CMakeLists.txt', example_files['/CMakeLists.txt']);
    fs.outputFileSync(compile_path + temp_dir + '/' + example_name + '/' + 'package.xml', example_files['/package.xml']);

    console.log("Compiling code.");
    var spawn = require('child_process').spawn;
    var command = './run_docker.sh'
    var docker_id = crypto.randomBytes(16).toString("hex");
    var net_id = crypto.randomBytes(16).toString("hex");
    var args = ['10', docker_id, 'ros_workspace_rosbridge', compile_path + temp_dir  + '/', net_id];
    var ls = spawn(command, args);

    ls.stdout.on('data', (data) => {
      //console.log(`stdout: ${data}`);
      // TODO: Actually it would be great if the text was rendered in colour in the html page
      var data_string = data.toString().replace(/[\u001b\u009b][[()#;?]*(?:[0-9]{1,4}(?:;[0-9]{0,4})*)?[0-9A-ORZcf-nqry=><]/g, '');
      io.emit('compile_output', data_string);
    });

    ls.stderr.on('data', (data) => {
      console.log(`stderr: ${data}`);
      var data_string = data.toString().replace(/[\u001b\u009b][[()#;?]*(?:[0-9]{1,4}(?:;[0-9]{0,4})*)?[0-9A-ORZcf-nqry=><]/g, '');
      io.emit('compile_output', data_string);
    });

    ls.on('close', (code) => {
      console.log(`child process exited with code ${code}`);
      fs.removeSync(compile_path + temp_dir);
      res.end('true');
    });
});
