# Summary
This repository contains the code for the project for the Localization course in Udacity's Self-Driving Car Nanodegree.

## Overview
This was one of the most challenging coding assignments, because it was harder to find the cause of the issue if there were such occurences.
Some of the main challenges are described below:

## Coordinate space
This was one of the confusing aspects of the assignment.
It is stated that map coordinates are X = Right and Y = Down, in the lectures local car space is X = forward, Y = left.
Whereas in this task Readme it stated that Y positive = forward and X = right, essentially meaning 90 degree rotated axes.

## Debugging inaccurate results
There were few causes of inaccurate resutls, and with current outputs it was hard to understand whch part of the code is failing.
Especially initially, when there multiple issues with code, changing in one location to the correct solution gave worse overall error results, thus misleading about the possible fix.

I got over it by starting to log particles each step to the file and seeing intermediate results.

## Importance of noise 
In this assignment it was very important to add noise for the measurements. 
Uncertainty allows more particles to survive, and more importantly it allows to survive particles which are at the correct locations.
If there are particle with high confidence at the wrong location, then without uncertainly it quickly draws particles around that location and is not able to recover.

## Error threshold
If the mean weighted error got over ~0.1 in x or y direction, it was almost impossible to recover.
In case of such situation in the real life it would be reccomended to re-initialize from GPS or other source of localization.
Or as an alternative to re-initialize particles with much larger uncertainty to allow recovery.
