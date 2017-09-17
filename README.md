Online ROS
==================

This is a web system where the user can compile, edit and run working Robot Operating System (ROS) example code.

This code is currently in a pre alpha, proof of concept state. Do not use in a production environment.

The tutorials are located in a separate repository described below.

### Install (alpha)

You'll want to clone the code which runs the site and the example code into separate directories.

```
git clone https://github.com/davidmball/online_ros.git
git clone https://github.com/davidmball/ros_examples.git
```
Docker is required to run the example code. I use docker.ce. Install it with the instructions here:
https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/#install-docker-ce

Add your user to the `docker` group.
```
sudo usermod -a -G docker $USER
```
Then log out and back in in order to use docker with your user.

And get the docker container.
```
docker pull davidmball/online_ros:kinetic
```

Also, you will need npm/nodejs. Note that for the Google compute engine I had to use these commands to install it.
```
sudo apt-get install nodejs
sudo apt-get install npm
sudo ln -s /usr/bin/nodejs /usr/bin/node
```

Then install the package dependencies. Go to the online_ros directory and run
```
npm install
bower install
```

The Yjs server is needed. Install with
```
npm install -g y-websockets-server
```

### Running

#### Locally

You should edit the development config file to suit your system.

Run with:
```
nodemon -e js,html server.js
```
and in another terminal
```
y-websockets-server
```

#### Online

Set the node environment to production. You should edit the production config file to suit your system.
```
export NODE_ENV=production
```
On a production system I use pm2 to run both the online_ros site and y-websockets-server. I had to fiddle to give them access to port 80 and 443. I also have a cron job to, among other tasks, reset everything once a week.

### Docker container

The docker container is built automatically when there is a change to the dockerfile on master in github.

### Contributing to the web site

Pull requests are welcome. For consistent style use

[![JavaScript Style Guide](https://cdn.rawgit.com/standard/standard/master/badge.svg)](https://github.com/standard/standard)

Run with
```
standard --fix
```

### Other

If you want to test building the docker image locally then change to the dockerfile/kinetic directory and type
```
docker build -t ros-online-kinetic .
```

### License

The code is released under the BSD license.
Except where otherwise noted, the web pages and documentation are licensed under Creative Commons Attribution 3.0.
