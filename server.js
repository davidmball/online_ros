const http = require('http');
const fs = require('fs');
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

var example_path_name = 'topics';
var example_name = 'chatter';


var files = {};
dir.readFiles(example_path,
    function(err, content, filename, next) {
        if (err)
            throw err;

        files[filename.replace(example_path + example_path_name, '')] = content;
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
            data = data.toString().replace(/\{\{host_name\}\}/, host);
            data = data.toString().replace(/\{\{rosbridge_port\}\}/, rosbridge_port);

            res.send(data);
        }
    });
});

app.get("/get_files", function (req, res) {
  res.send(JSON.stringify(files));
});

var server = app.listen(host_port, function () {
   var host = server.address().address;
   var port = server.address().port;

   console.log("Example app listening at http://%s:%s", host, port);
})

var socket = io.listen(server);



app.post('/compile', function(req, res)
{
    console.log("Writing code.");
    fs.writeFileSync(compile_path + example_path_name + 'CMakeLists.txt', files['CMakeLists.txt']);
    fs.writeFileSync(compile_path + example_path_name + 'package.xml', files['package.xml']);
    for (var filename in req.body.code_files)
    {
      fs.writeFileSync(compile_path + example_path_name + filename, req.body.code_files[filename]);
    }
    console.log("Compiling code.");
    var spawn = require('child_process').spawn;
    var command = './run_docker.sh'
    var docker_id = crypto.randomBytes(16).toString("hex");
    var net_id = crypto.randomBytes(16).toString("hex");
    var args = ['10', docker_id, 'ros_workspace_rosbridge', compile_path, net_id];
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
