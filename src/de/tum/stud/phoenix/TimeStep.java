package de.tum.stud.phoenix;

import geometry_msgs.Pose;
import geometry_msgs.Twist;

public class TimeStep {
	Pose pose;
	Twist command;
	public TimeStep() {
		super();
		pose=ApplicationContext.newInstance(Pose.class, Pose._TYPE);
		command= ApplicationContext.newInstance(Twist.class, Twist._TYPE);
	}	
}