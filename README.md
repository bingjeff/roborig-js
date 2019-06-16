# roborig-js
Basic kinematics and dynamics for rigid-body chains written in javascript.

## Setup
This project is based around:

* Node.js
* Yarn
* Webpack
* Babel

I used the [following site](https://medium.com/front-end-weekly/what-are-npm-yarn-babel-and-webpack-and-how-to-properly-use-them-d835a758f987) as reference when setting this up.

## What is it?

Very loosely it is just a collection of Javascript classes to enable building kinematic-trees for rigid-body linkages. There are a few peculiarities:

* Each joint can only be "simple", i.e. pure rotation or pure translation.
* Kinematic partials are available up to 2nd order.
* Math relies on mathjs.
* Visualization relies on threejs.
