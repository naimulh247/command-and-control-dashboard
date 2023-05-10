#!/bin/bash

# Check if curl is installed
curl_installed=$(command -v curl)

if [[ -z $curl_installed ]]; then
    echo "curl not installed. Installing..."
    sudo apt-get update
    sudo apt-get install -y curl
else
    echo "curl is already installed."
fi

# Check if ros-noetic-rosbridge-server is installed
dpkg -s ros-noetic-rosbridge-server &> /dev/null

if [ $? -ne 0 ]; then
    echo "ros-noetic-rosbridge-server not installed. Installing..."
    sudo apt-get update
    sudo apt-get install -y ros-noetic-rosbridge-server
else
    echo "ros-noetic-rosbridge-server is already installed."
fi

# Check if nodejs and npm are installed
node_installed=$(command -v node)
npm_installed=$(command -v npm)

if [[ -z $node_installed ]] || [[ -z $npm_installed ]]; then
    echo "Node.js and/or npm not installed. Installing..."

    # Add Node.js repository for version 18.x
    curl -sL https://deb.nodesource.com/setup_18.x | sudo -E bash -

    # Install Node.js and npm
    sudo apt-get install -y nodejs

    # Check installed versions
    node_version=$(node --version)
    npm_version=$(npm --version)

    # Verify if the installed versions match the required versions
    if [[ $node_version != "v18.4.0" ]] || [[ $npm_version != "9.5.0" ]]; then
        echo "Node.js and/or npm version mismatch. Installing required versions..."
        sudo npm install -g n
        sudo n 18.4.0
        sudo npm install -g npm@9.5.0
    fi

else
    echo "Node.js and npm are already installed."
fi
