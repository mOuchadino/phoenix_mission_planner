package de.tum.stud.phoenix;
import java.awt.geom.Point2D;

public class Phoenix {
	
	public Phoenix() {
		super();
		setLocation(new Point2D.Double(0,0));
		setRoll(0.0);
		setNick(0.0);
		setYaw(0.0);
		globalYaw=0;
	}

	//estimated pose
	private Point2D location;
	private double roll;
	private double nick;
	private double yaw;
	private double globalYaw;
	
	public double getGlobalYaw() {
		return globalYaw;
	}
	
	public void resetGlobalYaw() {
		globalYaw=0;
	}

	public void setLocation(Point2D location) {
		this.location = location;
	}
	
	public Point2D getLocation() {
		return location;
	}
	
	public double getRoll() {
		return roll;
	}

	public void setRoll(double roll) {
		this.roll = roll;
	}

	public double getNick() {
		return nick;
	}

	public void setNick(double nick) {
		this.nick = nick;
	}

	public double getYaw() {
		return yaw;
	}

	public void setYaw(double yaw) {
		this.yaw = yaw;
		globalYaw+=yaw;
	}	
}