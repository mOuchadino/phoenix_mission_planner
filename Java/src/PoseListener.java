import java.awt.Point;
import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

/**
listens for new pose updates on /navdata in geometry_msgs/Twist form and applies them to the model.
When simulating a flight, we publish a geometry_msgs/Twist message on the /cmd_vel topic but this time with the commanded speeds and angles.
 */

public class PoseListener extends AbstractNodeMain {
	private Phoenix phoenix;

	public PoseListener(Phoenix phoenix) {
		this.phoenix=phoenix;
	}

	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("PoseListener");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		final Log log = connectedNode.getLog();
		Subscriber<geometry_msgs.Twist> subscriber = connectedNode.newSubscriber("navdata", geometry_msgs.Twist._TYPE);
		subscriber.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
			@Override
			public void onNewMessage(geometry_msgs.Twist twist) {
				phoenix.setLocation(new Point(twist.getData().linear[0],new Point(twist.getData().linear[0])));
				phoenix.setYaw(twist.getData().angular[0]);
			}
		});
	}
}