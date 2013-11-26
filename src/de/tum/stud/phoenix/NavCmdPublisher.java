package de.tum.stud.phoenix;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

/**
When flying we publish a geometry_msgs/Twist message
on the /cmd_vel topic
with the desired linear and angular velocities
additionally the phoenix will only accept the commands if sim==false
 */

public class NavCmdPublisher extends AbstractNodeMain {
	private Trajectory trajectory;
	private int hz; //publishing frequency in hz
	private int timeStep;
	private boolean simulation;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("NavCmdPublisher");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		ApplicationContext.put(ConnectedNode.class,connectedNode);
		final Publisher<geometry_msgs.Twist> twistPublisher = connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
		final Publisher<std_msgs.Bool> boolPublisher = connectedNode.newPublisher("sim", std_msgs.Bool._TYPE);

		connectedNode.executeCancellableLoop(new CancellableLoop() {

			@Override
			protected void setup() {
			}

			@Override
			protected void loop() throws InterruptedException {
				if ( trajectory == null ) {
					Thread.sleep(1000/hz);
					return;
				}	else {
					if (timeStep<trajectory.segments.size()){
						geometry_msgs.Twist command = twistPublisher.newMessage();
						std_msgs.Bool simCommand = boolPublisher.newMessage();
						
						command.setLinear(trajectory.getCommand(timeStep).getLinear());
						command.setAngular(trajectory.getCommand(timeStep).getAngular());
						
						if (simulation) {
							simCommand.setData(true);
						} else {
							simCommand.setData(false);
						}

						twistPublisher.publish(command);
						boolPublisher.publish(simCommand);
						timeStep++;
					}

					Thread.sleep(1000/hz);
				}			
			}
		});
	}

	public NavCmdPublisher() {
		super();
		ApplicationContext.put(NavCmdPublisher.class, this);
		this.hz=1;
	}

	public void setTrajectory(Trajectory trajectory) {
		this.trajectory = trajectory;
		timeStep=0;
	}

	public void setHz(int hz) {
		this.hz = hz;
	}

	public void enableSimulation() {
		simulation=true;
	}

	public void disableSimulation() {
		simulation=false;
	}
}