package de.tum.stud.phoenix;

import org.ros.internal.message.RawMessage;

import geometry_msgs.Vector3;

public class MyVector3 implements Vector3 {
	double x,y,z;

	@Override
	public RawMessage toRawMessage() {
		return null;
	}

	@Override
	public double getX() {
		return x;
	}

	@Override
	public double getY() {
		return y;
	}

	@Override
	public double getZ() {
		return z;
	}

	@Override
	public void setX(double arg0) {
		x=arg0;
	}

	@Override
	public void setY(double arg0) {
		y=arg0;
	}

	@Override
	public void setZ(double arg0) {
		z=arg0;
	}

}
