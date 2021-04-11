# Robotic Arm
<img src="https://github.com/jdh0312/Configuration-space-planning/blob/main/result.jpg" width="40%"></img> 
<img src="https://github.com/jdh0312/Configuration-space-planning/blob/main/resultMaze.jpg" width="20%"></img> 

## Goal:
Transform a 2D planning problem for a robotic arm into a configuration space, and then search for a path in that space.

## Requirements:
```
python3
pygame
numpy (optional)
```
## Running:
The main file to run the mp is mp1.py:

```
usage: mp2.py [-h] [--map MAP_NAME] [--method {bfs}]
              [--human] [--fps FPS] [--granularity GRANULARITY]
              [--trajectory TRAJECTORY] [--save-image SAVEIMAGE]
              [--save-maze SAVEMAZE]
```

Examples of how to run MP2:
```
python3 mp2.py --map Map1 --human
```


For help run:
```
python3 mp2.py -h
```
Help Output:
```
CS440 MP2 Robotic Arm

optional arguments:
  -h, --help            show this help message and exit
  --map MAP_NAME        configuration filename - default BasicMap
  --method {bfs}        search method - default bfs
  --human               flag for human playable - default False
  --fps FPS             fps for the display - default 30
  --granularity GRANULARITY
                        degree granularity - default 2
  --trajectory TRAJECTORY
                        leave footprint of rotation trajectory in every x
                        moves - default 0
  --save-image SAVEIMAGE
                        save output to image file - default not saved
  --save-maze SAVEMAZE  save the contructed maze to maze file - default not
                        saved

```

You can run the following command to generate maze and trajectory, which should look similar to thosee in the folder "SampleOutputs"
```
python3 mp2.py --map Test1 --granularity=2 --trajectory=1 --method=bfs --save-image=test1.png --save-maze=test1.txt
python3 mp2.py --map Test2 --granularity=2 --trajectory=2 --method=bfs --save-image=test2.png --save-maze=test2.txt
python3 mp2.py --map BasicMap --granularity=2 --trajectory=1 --method=bfs --save-image=basicmap.png --save-maze=basicmap.txt
```
