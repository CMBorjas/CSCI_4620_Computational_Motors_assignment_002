import opensim as osim
import sys
import math
import numpy as np

model_filename = 'ue_torque.osim'


model = osim.Model(model_filename)
stepsize = 0.005;
nbr_timesteps = 150;

state = model.initSystem()
muscleController = osim.PrescribedController();
muscleSet = model.getMuscles();
torqueSet = model.getActuators();
nbr_muscles = muscleSet.getSize();

for j in range(torqueSet.getSize()):
	func = osim.Constant(1.0)
	muscleController.addActuator(torqueSet.get(j))
	muscleController.prescribeControlForActuator(torqueSet.get(j).getName(), func)
model.addController(muscleController)

model.setUseVisualizer(True)

state = model.initSystem() 
coords = model.updCoordinateSet();

model.equilibrateMuscles(state)
manager = osim.Manager(model)
manager.initialize(state)

f = 0;

total_time = stepsize * nbr_timesteps;

com_i = model.calcMassCenterPosition(state);
max_com = -math.inf;



for i in range(0,nbr_timesteps+1):
	t = state.getTime() + stepsize
	state = manager.integrate(t)
		
	brain = osim.PrescribedController.safeDownCast(model.getControllerSet().get(0))
	functionSet = brain.get_ControlFunctions()

	shoulder_elv = coords.get('shoulder_elv').getValue(state);
	shoulder_elv_v = coords.get('shoulder_elv').getSpeedValue(state);

	#INCLUDE CODE FOR ELBOW. 
	#The angle is elbow_flexion.
	elbow_flexion = coords.get('elbow_flexion').getValue(state);
	elbow_flexion_v = coords.get('elbow_flexion').getSpeedValue(state);
	target_angle = 1.0;
	
	for j in range(functionSet.getSize()):
		func = osim.Constant.safeDownCast(functionSet.get(j))
		
		if j == 1:
			func.setValue(-20 * (shoulder_elv - 1.5708) - 1 * shoulder_elv_v)
		elif j == 3:
		#	INCLUDE CODE FOR ELBOW  
			func.setValue(-15 * (elbow_flexion - target_angle) - 1 * elbow_flexion_v)


	model.realizeDynamics(state);
	com = model.calcMassCenterPosition(state);


statesDegrees = manager.getStateStorage();
statesDegrees.printToFile("PDControl.sto", "w");

