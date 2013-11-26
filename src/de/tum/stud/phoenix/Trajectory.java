package de.tum.stud.phoenix;

import geometry_msgs.Pose;
import geometry_msgs.Twist;

import java.util.ArrayList;

//A trajectory is a path parametrized by time
//for each time step give back assumed position and the necessary twist command to reach the next timestep

public class Trajectory {
		
	ArrayList<TimeStep> segments;
	
	public Trajectory() {
		super();
		segments=new ArrayList<TimeStep>();
	}

	public Pose getPose(int t){
		return segments.get(t).pose;
	}
	
	public Twist getCommand(int t){
		return segments.get(t).command;
	}
}
