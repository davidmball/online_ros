By Working Example
==================

This is a web system where the user can compile, edit and run working example code.

This code is currently in a pre alpha, proof of concept state. Do not use in a production environment.

At the moment this only targets Robot Operating System (ROS).

The tutorials are located in a separate repository which you can get from here : .

### Install (alpha)

You'll need git.
```
sudo apt-get install git
```
You'll want to clone the code which runs the site and the example code into separate directories.
```
git clone https://github.com/davidmball/by_working_example.git
git clone https://github.com/davidmball/ros_examples.git
```
Docker is required to run the example code. For the Google compute engine I use docker.io.
```
sudo apt-get install docker.io
```

And get the docker container. This isn't public yet.

Also, you will need npm/nodejs. Note that for the Google compute engine I had to use these commands to install it.
```
NODE_VERSION="v4.2.6"
curl -LO http://nodejs.org/dist/$NODE_VERSION/node-$NODE_VERSION-linux-x64.tar.gz
tar xzf node-$NODE_VERSION-linux-x64.tar.gz
sudo cp -rp node-$NODE_VERSION-linux-x64 /usr/local/
sudo ln -s /usr/local/node-$NODE_VERSION-linux-x64 /usr/local/node
nodejs --version
```

Then install the package dependencies.

### Running

Set the correct configuration to either production or development. You should edit the development config file to suit your system.
```
export NODE_ENV=production
```

Then run locally with:
```
nodemon server.js
```
Or on a production system at the moment to get access to port 80.
```
sudo -E bash -c '/usr/local/node/bin/node server.js'
```

### License

The code is released under the BSD license.
Except where otherwise noted, the web pages and documentation are licensed under Creative Commons Attribution 3.0.
