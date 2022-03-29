# Custom Raycast based vehicle implementation for Godot 3.x

## This branch is now stale. There is now an [experimental branch](https://github.com/Tobalation/GDCustomRaycastVehicle/tree/sphere-cast) that uses shape casting instead.

![icon](https://github.com/Tobalation/RaycastVehicleTest/blob/master/icon.png)

A custom raycast vehicle implementation emphasizing simplicity and adaptability.

The raycast elements serve as a basis for any type of land vehicle that needs to be suspended by "springs" and propelled
using driving force applied to the vehicle body against the a surface that provides traction (the ground).

This system primarily allows for the simulation of wheeled, tracked and hover vehicles with as many propulsion elements as needed.

## In this project:

Currently, two different sample vehicles are provided:

1. A tracked vehicle with 10 drive elements total (5 per track on each side) with a steering system that has neutral steer capability.

2. A 4 wheel drive vehicle with 4 drive elements (1 per wheel) with traditional car steering.

The vehicle controllers for both vehicle is only a bare minimum. Realistic drivetrain, sound, animation and traction are not simulated but can definitely be added.

The default scene is the tracked vehicle. You can switch to the other vehicle by pressing escape to unlock the mouse
and clicking on the 'Change to 4WD vehicle' button.
