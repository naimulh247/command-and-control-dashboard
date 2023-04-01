# Campus Rover Web Interface
This website allows users to connect to the campus rover and control it. There is also a live video feed.
## Package Dependencies
Connecting to the TurtleBot requires that the turtlebot is running a package called `rosbridge_server`. This package will spin up a websocket serveron the TurtleBot, allowing for outside connections to communicate with the robot.
## Running the website
1. Clone repo
2. Navigate to the website directory from your terminal
3. `npm install`
4. `npm start`

## Connecting to the TurtleBot
##### On the TurtleBot:
1. Start running `rosbridge_server`
* This can be done using the `rosbridge_websocket.launch` file
##### On Your Machine:
1. Start up the website (See steps above)
2. After starting the rosbridge server, get the ip address of the server from the TurtleBot (this can be seen in the terminal after running the rosbridge launch file)
3. Enter the IP address on the website and press connect

