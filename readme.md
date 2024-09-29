# Autonomous Robot Mop Web Application

Welcome to the Autonomous Robot Mop Web Application! This project allows you to control an autonomous robot mop using a sleek, modern web interface. The application includes features like real-time mapping, manual control with a joystick, settings configuration, and scheduling cleaning routines.

This guide will walk you through installing all the necessary components to get the application up and running.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation Overview](#installation-overview)
- [Backend Setup](#backend-setup)
  1. [Install System Dependencies](#1-install-system-dependencies)
  2. [Set Up Python Virtual Environment](#2-set-up-python-virtual-environment)
  3. [Install Backend Python Packages](#3-install-backend-python-packages)
  4. [Initialize the Database](#4-initialize-the-database)
  5. [Run the Backend Server](#5-run-the-backend-server)
- [Frontend Setup](#frontend-setup)
  1. [Install Node.js and npm](#1-install-nodejs-and-npm)
  2. [Install Frontend Dependencies](#2-install-frontend-dependencies)
  3. [Run the Frontend Development Server](#3-run-the-frontend-development-server)
- [ROS2 Integration](#ros2-integration)
  1. [Install ROS2](#1-install-ros2)
  2. [Install rosbridge_server](#2-install-rosbridge_server)
  3. [Run rosbridge_server](#3-run-rosbridge_server)
- [Running the Application](#running-the-application)
- [Project Structure](#project-structure)
- [Additional Notes](#additional-notes)
- [Troubleshooting](#troubleshooting)
- [License](#license)
- [Contact](#contact)

## Prerequisites
- **Operating System**: Ubuntu 22.04 LTS (or Ubuntu 24.04 if available)
- **Python**: 3.8 or higher
- **Node.js**: 14.x or higher
- **npm**: 6.x or higher
- **ROS2**: Humble Hawksbill (or latest version)
- **Git**: For cloning the repository

## Installation Overview
The installation involves setting up both the backend and frontend components, as well as integrating ROS2 for robot communication.

- **Backend**: A Flask server with a RESTful API to handle shared settings and schedules.
- **Frontend**: A React application with enhanced UI, including icons and improved page integration.
- **ROS2 Integration**: Using rosbridge_server for WebSocket communication between ROS2 and the web app.

## Backend Setup

### 1. Install System Dependencies
Open a terminal and update your package list:
```bash
sudo apt update
sudo apt install python3-pip python3-venv git


# Update package list
sudo apt update

# Install Python and virtual environment
sudo apt install python3-pip python3-venv

# Install Node.js and npm
sudo apt install nodejs npm

# Install pip dependencies
pip install -r requirements.txt

# Install ROS2
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install ros-jazzy-desktop

# Source ROS2 setup script
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

robot_ui/
├── public/
│   └── index.html
├── src/
│   ├── components/
│   │   ├── Navbar.js
│   │   ├── MapView.js
│   │   ├── ControlPage.js
│   │   ├── SettingsPage.js
│   │   └── SchedulingPage.js
│   ├── App.js
│   ├── index.js
│   └── styles/
│       └── styles.css
├── package.json
└── ...

## Starting the Web Server

1) Running the rosbridge_server

ros2 launch rosbridge_server rosbridge_websocket_launch.xml

2) Running the backend app

cd robomop_web/robot_ui/backend
python app.py

3) Running the frontend ui

cd robomop_web/robot_ui/src
npm start