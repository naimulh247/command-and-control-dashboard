# Deploying/ Running the Web-client in VNC/ Cloud environment

This guide will walk through deploying the web-client dashboard with `npm run start`. Follow the steps below to get the dashboard up and running.

## Prerequisites

Ensure the following is installed on your system using the [`setup.sh`](./setup.sh):

- Node.js (v18.4.0 recommended)
- npm (v9.5.0 recommended)

Run `bash setup.sh` if you are unsure if its installed

## Step 1: Install Dependencies

Navigate to the **web-client** directory. 
```bash
cd web-client
```

Run the following command to install the required dependencies:

```bash
npm i
```

This command installs all the packages listed in the `dependencies` sections of `package.json` file.

## Step 2: Start the Development Server

To start the server, run the following command:
```bash
npm run start
```

You should now be able to access the dashboard at [localhost:3000](http://localhost:3000)

There are two deployments of the dashboard, one in [Azure](http://40.76.113.205:3000/) and one in [Naimul's VNC](http://vnc.naimulhasan.ros.campusrover.org/)

## Connecting the robot

Make sure you have **rosbridge-server** installed on the robot and running `real.launch` using roslaunch after building it in your ros workspace

```bash
roslaunch ros-robot real.launch
```

In the dashboard's settings page, enter the IP address of the robot. By default, rosbridge-server runs on port 9090. Feel free to change any other settings you desire. 

After saving the settings, the status bar should change green if it successfully connects to the robot