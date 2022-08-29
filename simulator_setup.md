# Simulator setup


## Clone the simulator

Clone git:

```
$ mkdir ~/workspace
$ cd workspace
$ git clone https://github.com/samuelbortolin/AppliedRoboticsEnvironment.git simulator/
```

Compile the simulator:

```
$ cd simulator
$ catkin build
$ source ./environment.sh # aliases definition
```

## Clone the project

Clone forked git:

```
$ cd workspace
$ git clone https://github.com/samuelbortolin/AppliedRoboticsStudentInterface.git project/
```

Compile the project:

```
$ cd project
$ mkdir build		# call this folder build or the system will not work
$ cd build
$ cmake ..		# be sure that before you: source ~/workspace/simulator/environment.sh
$ make			# compile the .cpp file contained in ~/project/src. The cmake is including the header in ~/
$ source ../environment.sh
```


## Automatic source of the environment

Do this only if the above procedure worked without errors.

```
echo "source ${AR_CATKIN_ROOT}/environment.sh" >> ~/.bashrc
echo "source ${AR_ROOT}/environment.sh > /dev/null 2>&1" >> ~/.bashrc
```

