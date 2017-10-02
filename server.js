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

const http = require('http')
const https = require('https')
const fs = require('fs-extra')
const express = require('express')
const io = require('socket.io')(http)
const dir = require('node-dir')
const bodyParser = require('body-parser')
const crypto = require('crypto')
const config = require('config')
const parseString = require('xml2js').parseString
const helmet = require('helmet')
const uniqueRandomAtDepth = require('unique-random-at-depth')
const path = require('path')
const stream = require('tailing-stream')

var options
if (process.env.NODE_ENV === 'production') {
  options = {
    key: fs.readFileSync('/etc/letsencrypt/live/www.onlineros.com/privkey.pem'),
    cert: fs.readFileSync('/etc/letsencrypt/live/www.onlineros.com/fullchain.pem')
  }
}

var app = express()
app.use(helmet())
app.use(express.static('public'))
app.use(bodyParser.json())
app.use(bodyParser.urlencoded({extended: true}))

const examplePath = config.get('example_path')
const compilePath = config.get('compile_path')
const hostPort = config.get('host_port')
const hostName = config.get('host')

// Within the ephemeral port range
// I'll have other problems before I run out of these ports :)
const rosbridgePortMin = 50000
const rosbridgePortMax = 59999

var files = []
var exampleFiles = []
var exampleList = []

var htmlFooterFile
var htmlHeaderFile
var htmlHeadFile
var indexFile
var contributeFile
var aboutFile

fs.readFile('html_footer.html', 'utf8', function (err, data) {
  if (err) throw err
  htmlFooterFile = data
})
fs.readFile('html_header.html', 'utf8', function (err, data) {
  if (err) throw err
  htmlHeaderFile = data
})
fs.readFile('html_head.html', 'utf8', function (err, data) {
  if (err) throw err
  htmlHeadFile = data
})
fs.readFile('html_googleanalytics.html', 'utf8', function (err, data) {
  if (err) throw err
  htmlGoogleAnalyticsFile = data
})

/**
 * Runs at the start to read, process and store all of the example files.
 */
dir.readFiles(examplePath, function (err, content, filename, next) {
  if (err) { throw err }

  var filename2 = filename
  var exampleName = filename.replace(examplePath, '').split('/')[0]
  var exampleFilename = filename2.replace(examplePath + exampleName, '')

  if (exampleFilename === '/package.xml') {
    parseString(content, function (err, result) {
      if (err) { throw err }
      exampleFiles.push(result)
    })
  }
  files.push([exampleName, exampleFilename, content])
  next()
},
    function (err) {
      if (err) { throw err }
      console.log('Read this many examples:', exampleFiles.length)

        // store the package and user configuration xml for the index page
      for (var i = 0; i < exampleFiles.length; i++) {
        var exampleXML
        for (var j = 0; j < files.length; j++) {
          if (files[j][0] === exampleFiles[i].package.name + '.xml') { exampleXML = files[j][2] }
        }
        var resultXML = ''
        parseString(exampleXML, function (err, result) {
          if (err) { throw err }
          resultXML = result
        })

        var languages = ""
        for (var j = 0; j < resultXML.example.language.length; j++) {
          languages = languages.concat(resultXML.example.language[j].name + ' ')
        }

        exampleList.push([exampleFiles[i], resultXML, languages])
      }
    })

io.on('connection', function (socket) {
  console.log('a user connected')
})

function filter(val) {
  if (typeof val === 'undefined')
    return ""
  else
    return val

}

function getIndexList()
{
  data = "";
  for (i = 0; i < exampleList.length; i++)
  {
    // TODO: Clean up this div using a style sheet, etc
    data += "<div class=\"card\">"
    data += "<a href=\"example.html?name=" + exampleList[i][0].package.name + "\"><span class=\"link-spanner\"></span></a>\n"
    data += "<b><span style=\"font-size: 1.4em; color: #0000BB;\">" + exampleList[i][1].example.title + " </span></b>\n"
    data += "<i>(tags: " + exampleList[i][1].example.tag + ")</i>\n"
    data += "<span style=\"float: right\">" + exampleList[i][1].example.ros_version + "</span></br>"
    data += exampleList[i][1].example.description + "</br>\n"
    data += "<span style=\"position: absolute; bottom: 0;\">Packages: " + filter(exampleList[i][0].package.run_depend) + " " + filter(exampleList[i][0].package.depend) + "</span>\n"
    data += "<span style=\"position: absolute; bottom: 0; right: 0; background: lightgreen;\">" + filter(exampleList[i][2]) + "</span>\n"
    data += "</div>\n\n"
  }

  return data;
}

function replaceTemplates (data, name, title, description) {
  data = data.toString().replace(/\{\{html_footer\}\}/g, htmlFooterFile)
  data = data.toString().replace(/\{\{html_header\}\}/g, htmlHeaderFile)
  data = data.toString().replace(/\{\{html_head\}\}/g, htmlHeadFile)

  if (process.env.NODE_ENV === 'production') {
      data = data.toString().replace(/\{\{google_analytics\}\}/g, htmlGoogleAnalyticsFile)
  } else {
      data = data.toString().replace(/\{\{google_analytics\}\}/g, '')
  }

  // Set the title and description which is what google uses for search results.
  const siteTitle = 'Online ROS'
  if (name === '/example.html') {
    data = data.toString().replace(/\{\{title\}\}/g, siteTitle + ' - Example: ' + title)
    data = data.toString().replace(/\{\{description\}\}/g, description)
  } else {
    if (name === '/index.html') { data = data.toString().replace(/\{\{title\}\}/g, siteTitle + ' - Home') }
    if (name === '/about.html') { data = data.toString().replace(/\{\{title\}\}/g, siteTitle + ' - About') }
    if (name === '/contribute.html') { data = data.toString().replace(/\{\{title\}\}/g, siteTitle + ' - Contribute') }

    data = data.toString().replace(/\{\{description\}\}/g, 'View, edit, compile and run ROS examples in your browser for free.')
  }

  if (name === '/index.html') {
    data = data.toString().replace(/\{\{example_list\}\}/g, getIndexList())
  }

  return data
};

// Redirect root to index page
app.get('/', (req, res)=>{
  res.redirect('/index.html')
})

/**
 * This returns the example page with the rosbridge url and port set.
 */
app.get('/example.html', function (req, res) {
  // TODO: Handle when the example requested isn't in the list. To begin with
  // redirect the user back to the index page.
  if (typeof req.query.name === 'undefined' || !req.query.name) {
    res.redirect('index.html')
  } else {
     // Set the web address and port for the rosbridge api call.
    fs.readFile(path.join(__dirname, 'example.html'), function (err, data) {
      if (err) {
        res.send(404)
      } else {
        res.contentType('text/html')
              // the last param is the repeat depth
        var rosbridgePort = uniqueRandomAtDepth(rosbridgePortMin, rosbridgePortMax, 5000)
        data = data.toString().replace(/\{\{host_name\}\}/g, hostName)
        data = data.toString().replace(/\{\{rosbridge_port\}\}/g, rosbridgePort())

        // TODO: Cache this lookup!
        var exampleName = req.query.name
        var exampleXML
        for (var j = 0; j < files.length; j++) {
          if (files[j][0] === exampleName + '.xml') { exampleXML = files[j][2] }
        }
        var resultXML = ''
        parseString(exampleXML, function (err, result) {
          if (err) { throw err }
          resultXML = result
        })

        res.send(replaceTemplates(data, '/example.html', resultXML.example.title, resultXML.example.description))
      }
    })
  }
})

/**
 * This caches file to save reloading from disk.
 */
function cacheFile (filename, file, res) {
  if (typeof file === 'undefined') {
    fs.readFile(path.join(__dirname, filename), function (err, data) {
      if (err) {
        res.send(404)
      } else {
        var fileData = replaceTemplates(data, filename)
        res.contentType('text/html')
        res.send(fileData)
        return fileData
      }
    })
  } else {
    res.contentType('text/html')
    res.send(file)
    // TODO: file is already set so we shouldn't have to return anything here.
    return file
  }
}

app.get('/index.html', function (req, res) {
  indexFile = cacheFile('/index.html', indexFile, res)
})

app.get('/contribute.html', function (req, res) {
  contributeFile = cacheFile('/contribute.html', contributeFile, res)
})

app.get('/about.html', function (req, res) {
  aboutFile = cacheFile('/about.html', aboutFile, res)
})

/**
 * This returns the code / other files for a particular example and is
 * called when the user opens the examples page.
 */
app.get('/get_files', function (req, res) {
  // TODO: Handle when the example requested isn't in the list.
  var exampleName = req.query.name
  var exampleFiles = {}
  for (var i = 0; i < files.length; i++) {
    if (files[i][0] === exampleName) { exampleFiles[files[i][1]] = files[i][2] }
  }

  // get the xml data.
  // TODO: This lookup should be generated and stored globally.
  var exampleXML
  for (var j = 0; j < files.length; j++) {
    if (files[j][0] === exampleName + '.xml') { exampleXML = files[j][2] }
  }
  var resultXML = ''
  parseString(exampleXML, function (err, result) {
    if (err) { throw err }
    resultXML = result
  })

  res.send(JSON.stringify([exampleFiles, resultXML]))
})


var http_server = http.createServer(app).listen(hostPort)
var https_server
if (process.env.NODE_ENV === 'production') {
//  https_server = https.createServer(options, app).listen(443)
}

io.listen(http_server)

/**
 * Takes the code files from the user, compiles and runs them in a docker container,
 * and returns the std outputs to the user.
 * Overwrites the package.xml and CMakeLists.txt file with the original. (For security.)
 */
app.post('/compile', function (req, res) {
  // each compile get written to its own directory
  var tempDockerDir = crypto.randomBytes(16).toString('hex')

  // write out the user files to the directory taht will be shared with the docker container
  // except CMakeLists and package.
  var exampleName = req.query.name
  for (var filename in req.body.code_files) {
    if (filename === '/CMakeLists.txt' || filename === '/package.xml') { continue }
    fs.outputFileSync(compilePath + tempDockerDir + '/' + exampleName + '/' + filename, req.body.code_files[filename])
    // to make sure the python files are executable
    fs.chmod(compilePath + tempDockerDir + '/' + exampleName + '/' + filename, '0755')
  }

  // write out the CMakeLists and packge files.
  // TODO: These lookups should be pre-generated and stored globally.
  var exampleFiles = {}
  var exampleXML
  for (var i = 0; i < files.length; i++) {
    if (files[i][0] === exampleName) { exampleFiles[files[i][1]] = files[i][2] }

    if (files[i][0] === exampleName + '.xml') { exampleXML = files[i][2] }
  }
  fs.outputFileSync(compilePath + tempDockerDir + '/' + exampleName + '/' + 'CMakeLists.txt', exampleFiles['/CMakeLists.txt'])
  fs.outputFileSync(compilePath + tempDockerDir + '/' + exampleName + '/' + 'package.xml', exampleFiles['/package.xml'])

  // setup and spawn the docker container
  var spawn = require('child_process').spawn
  var dockerID = crypto.randomBytes(16).toString('hex')
  var netID = crypto.randomBytes(16).toString('hex')
  var runCommand = ''
  var rosbridgePort = req.body.rosbridge_port
  var runTime = 15
  parseString(exampleXML, function (err, result) {
    if (err) { throw err }

    for (var i = 0; i < result.example.language.length; i++) {
      if (JSON.stringify(req.body.language) === JSON.stringify(result.example.language[i].name[0])) {
        runCommand = result.example.language[i].run_cmd
      }
    }
    runTime = result.example.time_limit
  })

  fs.outputFileSync(compilePath + tempDockerDir + '/' + 'stdouterr.txt', '')
  var readStream = stream.createReadStream(compilePath + tempDockerDir + '/' + 'stdouterr.txt', {encoding: 'utf8', timeout: 30000, interval: 10})

  console.log('Compiling and running code: ' + runCommand)
  var args = [runTime, dockerID, 'davidmball/online_ros:kinetic', compilePath + tempDockerDir + '/', netID, runCommand, rosbridgePort]
  var ls = spawn('./run_docker.sh', args)

  readStream.on('data', chunk => {
    // TODO: It would be great to keep the coloured output.
    var dataString = chunk.toString().replace(/[\u001b\u009b][[()#;?]*(?:[0-9]{1,4}(?:;[0-9]{0,4})*)?[0-9A-ORZcf-nqry=><]/g, '')
    io.emit('compile_output' + rosbridgePort, dataString)
  })
  // quietly handle the error when deleting the file before the stream timeouts
  readStream.on('error', function() {})

  // end this post request
  ls.on('close', (code) => {
    fs.removeSync(compilePath + tempDockerDir + '/' + 'stdouterr.txt')
    fs.removeSync(compilePath + tempDockerDir)
    res.end('true')
  })
})
