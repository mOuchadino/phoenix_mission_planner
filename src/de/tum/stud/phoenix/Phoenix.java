package de.tum.stud.phoenix;

import geometry_msgs.Point;
import geometry_msgs.Pose;

//target speed 0.5 m/s

public class Phoenix {
	private Pose pose = null;

	public Phoenix() {
		super();
	}

	public Pose getPose() {
		if ( pose == null )
			pose=ApplicationContext.newInstance(Pose.class, Pose._TYPE);
		return pose;
	}

	public void setPose(Pose pose) {
		this.pose = pose;
	}

	public void setLocation(Point currentPoint) {
		pose.setPosition(currentPoint);
	}

	public void resetGlobalYaw() {
	}
}