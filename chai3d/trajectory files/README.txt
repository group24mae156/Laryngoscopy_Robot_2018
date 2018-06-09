This is the default location of where trajectory files are read from and written to

Reading Trajectory


Modes: {test, multi, else}

test: This mode has prepogrammed in files to be read from, file to be written to, and proportional constant.
Instructions: 
1. Choose mode
2. Run program


multi: This mode is used for loading multiple trajectories quickly
Instructions: 
1. Choose mode
2. Create folder inside trajectory files folder
3. Place trajectoryfiles inside this folder with name "foldername + integer.m"
4. Enter folder name to be read when prompted
5. Enter file to record trajectory to
6. Enter proportional constant
7. Run program


else: This mode is trigured if anything besides test or multi is entered
Note: Trajectory files read here are read from the trajectory files folder, not their own folder.
Instructions: 
1. Choose mode
2. Choose trajectory files to be loaded, enter one at a time
3. Choose file to record trajectory to
4. Enter proportional constant
5. Run Program


Writing Trajectory
Trajectory data starts recording after pressing "r" and stops after again pressing "r".
To make sure data is saved, the program must be closed with "esc" or "q", closing by using "ctrl + c" will not save the data.
