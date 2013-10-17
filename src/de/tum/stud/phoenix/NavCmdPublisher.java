package de.tum.stud.phoenix;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.rosjava_geometry.Vector3;

/**
When simulating a flight, we publish a geometry_msgs/Twist message on the /cmd_vel topic with the desired velocities and angles.
 */

public class NavCmdPublisher extends AbstractNodeMain {
	private Phoenix phoenix;
	private int hz; //publishing frequency in hz

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("NavCmdPublisher");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		final Publisher<geometry_msgs.Twist> publisher = connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
		final Publisher<std_msgs.String> stringPublisher = connectedNode.newPublisher("cmd_vel2", std_msgs.String._TYPE);
		connectedNode.executeCancellableLoop(new CancellableLoop() {

			@Override
			protected void setup() {
			}

			@Override
			protected void loop() throws InterruptedException {
				if ( phoenix == null ) {
					System.out.println("Phoenix object not yet injected.");
					Thread.sleep(2000);
					return;
				}				
				geometry_msgs.Twist twist = publisher.newMessage();
				std_msgs.String str = stringPublisher.newMessage();

				
				Vector3 linear = new Vector3(phoenix.getLocation().getX(),phoenix.getLocation().getY(),0);
				Vector3 angular = new Vector3(0,0,phoenix.getYaw());
				MyVector3 lv3=  new MyVector3();
				MyVector3 av3=  new MyVector3();
				twist.setLinear(linear.toVector3Message(lv3));
				twist.setAngular(angular.toVector3Message(av3));
		        str.setData(String.format("Phoenix at %f %f \n", phoenix.getLocation().getX(),phoenix.getLocation().getY()));
				
				publisher.publish(twist);
		        stringPublisher.publish(str);

				System.out.format("Phoenix at %f %f \n", phoenix.getLocation().getX(),phoenix.getLocation().getY());
				Thread.sleep(1000/hz);
			}
		});
	}
	
	public NavCmdPublisher() {
		super();
		ApplicationContext.put(NavCmdPublisher.class, this);
		hz=1;
	}

	public void setPhoenix(Phoenix phoenix) {
		this.phoenix = phoenix;
	}

	public void setHz(int hz) {
		this.hz = hz;
	}
}