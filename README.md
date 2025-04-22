# CDE2310 AY 24/25 Group 5

## System requirements
This system uses a Turtlebot3 Burger (![specifications](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#specifications) can be found on the Robotis website)\
The robot is required to autonomously navigate and fully map a maze. Upon detecting a heat source, the robot should approach the heat source, stop, and fire 3 ping pong balls in a firing sequence of 2-4-2s. The robot must detect two distinct heat sources in the maze. Optionally, the robot can scale a ramp and detect a third heat source in the ramp zone.

The full final mission details can be found ![here](https://github.com/NickInSynchronicity/EG2310_AY2024-25/blob/main/docs/Mission%20Readme.md)

## Repository structure
```
— README.md
— Electronics
—— electronics_readme
— __pycache__
— assets
— .gitattributes
— __init__.py
— helper_funcs.py
— pathfinder.py
— mappingphase.py
— searchingphase.py
— survivorzonesequence.py
— package.xml
— dev_scripts
—— r2z1.py # for debugging 
—— ramp.py # ramp test script
—— sensor_development # scripts to test heat sensor
—— launcher_development # scripts used for development of launcher system
—— board_integration # other test scripts
— reference_scripts # Scripts used as reference, including original r2auto_nav scripts
— tasks # in-class tasks and submissions
```

## High-level design
Our group's system functions as follows:
![system flowchart](assets/FinalSolution.png)


## Subsystem design

### Mechanical

### Electrical
See ![electronics_readme](Electronics/electronics_readme.md)

### Software
See ![software_readme](software_readme.md)

## Testing documentation

## End user documentation
See ![user manual](assets/end_user_documentation_v1.1.pdf)
