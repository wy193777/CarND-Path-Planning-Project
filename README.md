# CarND-Path-Planning-Project

## Introduction

The goal of the project is to let a car drive itself in a simplified highway scenerio. The car doesn't need to care about any emergency condition like police or closed lane.

The car will keep getting sensor fusion data about surrounding vehicles. And the vehicle will process the data and issue control commands according to sensor fusion data around every 0.92 seconds.

There are 5 steps for every 0.92 seconds:

1. Transfer sensor fusion data into vehicle objects.

2. Predict where surrounding vehicles will be when the control command been executed.

3. Plan next possible moves.

4. Evaluate best move according to cost functions.

5. Generate trajectory for next move.

6. Send trajectory to controller.

## Details

### Transfer sensor fusion data into vehicle objects

The vehicle object contains car's 's', 'd' in frenet coordination.

### Predict where surrounding vehicles will be when the control command been executed

User object's current velocity and s to predict where the vehicle will be when the control command been executed.

### Plan next possible moves

The car can keep lane, prepare to change lane, change one lane or change two lanes. When consider if need to change lane, the car will take surrounding vehicles' position into consideration. If there are other vehicles on the intended lane or if there is a vehicle too close with car in the same lane, a change lane move will not be send to cost function to evaluate.

### Evaluate best move according to cost functions

After possible moves been decided, those moves will be send to cost function to evaluate. There basically 2 goals: if the vehicle is driving on:

1. highest possible speed (efficiency),
2. the central lane.

### Generate trajectory for next move

After determned next best move, then the car will generate a trajectory according to the move.

## Limitations

1. Sometimes the car will be trapped into most left or right lane if frond and side vehicles are slow.

2. Cannot do a low speed pass when all surrounding vehicles are slow.
