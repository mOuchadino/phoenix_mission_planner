import java.awt.Point;

public class Phoenix {
	
	public Phoenix() {
		super();
		setLocation(new Point(0,0));
		setRoll(0.0);
		setNick(0.0);
		setYaw(0.0);
		globalYaw=0;
	}

	//estimated pose
	private Point location;
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

	public void setLocation(Point location) {
		this.location = location;
	}
	
	public Point getLocation() {
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
		System.out.println("yaw is "+this.yaw);
		System.out.println("Globalyaw is "+this.globalYaw);
	}	
}