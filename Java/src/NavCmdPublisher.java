import geometry_msgs.Twist;
import geometry_msgs.Vector3;
import java.awt.Point;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

/**
When simulating a flight, we publish a geometry_msgs/Twist message on the /cmd_vel topic with the commanded velocoties and angles.
 */

public class NavCmdPublisher extends AbstractNodeMain {
	private Phoenix phoenix;
	private int hz; //publishing frequency in hz
	Vector3 linear;
	Vector3 angular;

	public NavCmdPublisher(Phoenix phoenix, int hz) {
		this.phoenix=phoenix;
		this.hz=hz;
	}

	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("NavCmdPublisher");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		final Log log = connectedNode.getLog();
		final Publisher<geometry_msgs.Twist> publisher = connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);

		connectedNode.executeCancellableLoop(new CancellableLoop() {
			private int sequenceNumber;

			@Override
			protected void setup() {
				sequenceNumber = 0;
			}

			@Override
			protected void loop() throws InterruptedException {
				geometry_msgs.Twist twist = publisher.newMessage();
				linear.setX(phoenix.getLocation().x);//provisorisch
				linear.setY(phoenix.getLocation().y);
				angular.setZ(phoenix.getYaw());
				twist.setLinear(linear);
				twist.setAngular(angular);
				publisher.publish(twist);
				sequenceNumber++;
				Thread.sleep(1000/hz);
			}
		});
	}
}