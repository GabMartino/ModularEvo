# ModularEvo

This project aims to develop a system that induces modularity in the Neural Network that controls simulated robots.

## Getting Started

---

### Prerequisites


Install MULTINEAT from https://github.com/ci-group/MultiNEAT . 
Install Gazebo-9.
Create a new virtual environment for python 3.x and...

```
pip install -r requirements.txt
```


Copy these two lines in ~/.bashrc changing "PathOfTheProject" into your folder

'''
	export GAZEBO_PLUGIN_PATH=~/PathOfTheProject/ModularEvo/build:$GAZEBO_PLUGIN_PATH
	export GAZEBO_MODEL_PATH=~/PathOfTheProject/ModularEvo/models:$GAZEBO_MODEL_PATH
'''

## Running and Test

1)Compile the plugins:

In the project folder:
'''
	mkdir build
	cd build
	cmake ../
	make
'''
2) Run the server:

'''
	cd models
	gzserver spider.world --verbose
'''
3) Run the Python Manager:

'''
	source venv/bin/activate
	python3 ExecuteTest.py
'''

Then you will see the progresses of the simulation in the "gzserver" shell.

If you want to activate the GUI, in a new shell:

'''
	gzclient
'''

## Authors

* **Gabriele Martino**

## Acknowledgments

--


