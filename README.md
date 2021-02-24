# kinematic_model

## Table of Contents
1. [Overview](#1-overview): Brief introduction to the kinematic_model package
2. [Details](#2-details): Detailed information about package features and functionality
3. [Installation](#3-installation): Instructions for installing the package
4. [Usage](#4-usage): Instructions for using the package

## 1. Overview

The kinematic_model package provides two primary functions:
1. Internally model the kinematics of a robot using an Unscented Kalman Filter (UKF)
2. Provide real-time calculations of transforms between coordinate frames (alternative to tf/tf2)

At a high level, this package allows a user to develop a very simple plugin that defines the geometry/kinematics of their robot. The user's plugin can then be loaded by the kinematic_model node, which internally handles state estimation and transform calculations.

**Features:**

- Plugin structure allow models to be easily created, loaded, and run for any type of robot
- Centralizes state estimation and transform calculation, increasing efficiency and reducing complexity of the running ROS environment
- Internally handles nonlinear state estimation for more accurate, real-time estimations of the robot's state (e.g. pose, joint positions, etc.)
- Extremely high-efficiency transform service for calculating transforms between coordinate frames

**Advantages over tf2:**

- Orders of magnitude increase in efficiency for both computation and bandwidth
- Nodes (especially sensor drivers) do not need to broadcast transform information and do not need awareness of robot geometry (e.g. frames)

**Tutorials:**

See the [kinematic_model_examples](https://github.com/pcdangio/ros-kinematic_model_examples) package for various examples using kinematic_model.

## 2. Details

The purpose of the kinematic_model package is to provide a plugin-based, centralized node for modeling the kinematic state of a robot in real time while simultaneously providing a service for geometric transforms.

At a high level:
1. The user develops a custom plugin for a robot that contains information about the robot's geometry and models it's kinematics
2. The user instructs the kinematic_model node on which plugin to use via a simple ROS parameter
3. The kinematic_node handles the state estimation, and simultaneously offers a get_transform ROS service

### 2.1: Plugins

Plugins have four primary responsibilities:

**Responsibility 1: Specify the robot geometry**

The kinematic_model node needs to understand the robot's geometry in order to calculate transformations between the robot's various frames. This package uses a special "design" structure that allows the plugin to create and connect geometry objects. These objects include:

- Links: A single, physical body within the robot.
- Joints: A dynamic connection between two links.
- Frames: A user-defined coordinate frame that can be attached to a link or joint.

These objects are connected via "attachments". An attachment may be fixed or dynamic, and there are no limitations on which objects can be attached together. For example, you may connect two links with a fixed attachment. You may also connect multiple joints together in series to create a 3DOF ball joint.

Joints and dynamic attachments must be tied to variables in the model's state vector. Take for example a revolute joint, which has a single axis of rotation. There must be a variable in the model's state vector that describes the angular position of that joint. This allows kinematic_model to internally calculate transforms using the real-time model state.

For simplicities sake, only two types of joints exist: revolute and prismatic. You may directly attach multiple joints together (without a link inbetween) to create more complex joints, such as a universal joint or a ball joint.

One may ask why kinematic_model doesn't use URDF or SDF for specifying robot geometry, and the answer is simple: both URDF and SDF use limiting and counter-intuitive methods for attaching geometry objects together. This package uses a custom procedure which drastically simplifies the process of "building" a robot geometry model while simultaneously allowing more flexibility.

**Responsibility 2: Model the robot's kinematics**

The kinematic_model node needs to understand the robot's kinematics in order to perform state estimation. This package internally uses an Unscented Kalman Filter (UKF) from the [kalman_filter](https://github.com/pcdangio/ros-kalman_filter) package for running state estimation calculations. The plugin must specify two different models:

1. State Transition Model: Predicts the current state of the model based on the previous state (and inputs if present)
2. Observation Model: Maps sensor measurements to variables in the model state.

These models are specified by overriding the plugin's base class methods for `state_transition()` and `observation()`. See the [Usage](#4-usage) section for more details.

The plugin's model functions may directly access transforms from it's base class using the `get_transform()` function. Many observation models require transformations to map sensor measurements to states, for example mapping a GPS measurement from the "gps_sensor" frame to a state represented in the "robot" frame.

Because the kinematic_model uses a UKF, it can handle both linear AND nonlinear models. It also doesn't require you to specify jacobians (like with the Extended Kalman Filter), drastically simplifying the model while still being able to handle nonlinearity.

The plugin is also responsible for setting the process covariance matrix, Q. The matrix is stored by the plugin's base class, and may be set once or modified at any time by the plugin.

**Responsibility 3: Provide the internal state estimator with sensor measurements**

The plugin must receive sensor measurements and pass them on as observations to the kinematic_model's state estimator. The reason for this is that each robot uses different sensors, so the robot-specific plugin is the correct place to subscribe to ROS sensor messages and parse them. Each time a new sensor measurement is received, the plugin must pass it into the state estimator via the `new_observation()` function. See the [Usage](#4-usage) section for more details.

A great feature of the kinematic_model's state estimator is that it can handle measurements that arrive at different/variable rates. The state estimator will use whatever measurements are available when performing its "measurement update" step.

An added bonus of this approach is that kinematic_model may use data from ROS sensor nodes without imposing any requirements on the sensor nodes themselves. In tf2, sensors are tasked with sending transform messages. This is a form of contamination, as a sensor node must programmatically depend on tf2 and also have knowledge of robot-specific coordinate frames. The kinematic_model package removes this contamination, making the ROS environment more elegant. It also means that kinematic_model can use data from ANY type of node.

The plugin is also responsible for setting the measurement noise covariance matrix, R. The matrix is stored by the plugin's base class, and may be set once or modified at any time by the plugin. Take for example the ROS [sensor_msgs/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html) message, which includes both a GPS measurement and covariance. In the subscriber's callback, your plugin may pass the measurement in via `new_observation()` and then update the R matrix using the real-time covariance measurements provided in the message.

The plugin may optionally use ROS messages to calculate or receive the initial state of the model (e.g. a service for setting the robot's initial state). The plugin may then use the base class's `initialize_state()` message to reset the state and associated covariance to the desired values.

**Responsibility 4: Publish state information externally as needed**

Each individual robot will have certain information about it's kinematic state that it may want to publish externally. For example, a robot may want to publish it's pose, or publish the real-time estimated state of each joint. This will differ for each robot; thus the plugin is the proper place for publishing custom ROS messages from.

The plugin may make use of the base plugin's `on_state_update()` method to publish information each time a new state is calculated, or it may use a ROS Timer to publish state information at different rates. See the [Usage](#4-usage) section for more details.

### 2.2: Base Class

The kinematic_model base class is the class that all plugins derive from. It contains all functionality related to state estimation, transform calculation, and serving transforms externally. It has three primary responsibilities:

**Responsibility 1: Continuously estimate the kinematic state of the robot**

The base class uses an internal Unscented Kalman Filter (UKF) from the ROS [kalman_filter](https://github.com/pcdangio/ros-kalman_filter) package for performing real-time, nonlinear state estimation. The state estimator runs at a predefined rate, which is specified as a simple ROS parameter. Upon each iteration, the state estimator:

1. Performs an prediction step using the plugin's state transition model.
2. Uses available measurements to perform a measurment update step.

Ultimately, this allows kinematic_model to calculate more accurate estimates of the actual kinematic state of the robot.

See the [kalman_filter](https://github.com/pcdangio/ros-kalman_filter) package for additional detail on how the internal state estimator works.

**Responsibility 2: Calculate transforms on an as-needed basis**

- kinematic_model transforms are calculated in one central location (tf2 requires each node to calculate it's own transforms - highly duplicative)
- custom graph implementation tailored specifically for high-efficiency path solving and transform chaining
- uses caching to further reduce repetitive path/transform calculations

**Responsibility 3: Provide an external ROS service for getting transforms**

- centralized configuration of kinematic_model minimizes data transfer between nodes 
- transforms are calculated and sent on-demand via a ROS service (tf2 continuously sends out every transform to every node at full rate)

### 2.3: Node

## 3. Installation

## 4. Usage