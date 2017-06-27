var http = require('http');
var fs = require('fs');
var express = require('express');
var path = require('path')
var io   = require('socket.io')(http);
var dir = require('node-dir');
var bodyParser = require('body-parser');

var app = express();
app.use(express.static('public'));
var jsonParser = bodyParser.json()
var urlencodedParser = bodyParser.urlencoded({ extended: false })


var example_name = 'chatter';
var files = {};
var path = '/home/david/ros_tutorials_ws/src/topics/';



dir.readFiles(path,
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
            data = data.toString().replace(/\{\{package_xml\}\}/, files['package.xml']);
            data = data.toString().replace(/\{\{cmakelists_txt\}\}/, files['CMakeLists.txt']);
            data = data.toString().replace(/\{\{chatter_cpp\}\}/, files['chatter.cpp']);

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


app.post('/compile', urlencodedParser, function(req, res)
{
    console.log("Writing code.");
    files['chatter.cpp'] = req.body.code;
    fs.writeFileSync('/home/david/catkin_ws/src/topics/CMakeLists.txt', files['CMakeLists.txt']);
    fs.writeFileSync('/home/david/catkin_ws/src/topics/package.xml', files['package.xml']);
    fs.writeFileSync('/home/david/catkin_ws/src/topics/src/chatter.cpp', files['chatter.cpp']);

    console.log("Compiling code.");
    var spawn = require('child_process').spawn;
    var command = './run_docker.sh'
    var args = ['10', 'fred', 'ros_workspace_rosbridge', '/home/david/catkin_ws/src'];
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
    });

});
