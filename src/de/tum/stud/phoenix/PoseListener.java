package de.tum.stud.phoenix;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

/**
listens for new pose updates on /navdata in geometry_msgs/Pose form and applies them to the model.
 */

public class PoseListener extends AbstractNodeMain {
	private Phoenix phoenix;

	public PoseListener() {
		super();
		ApplicationContext.put(PoseListener.class, this);
	}

	public void setPhoenix(Phoenix phoenix) {
		this.phoenix = phoenix;
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("PoseListener");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		Subscriber<geometry_msgs.Pose> subscriber = connectedNode.newSubscriber("navdata", geometry_msgs.Pose._TYPE);
		subscriber.addMessageListener(new MessageListener<geometry_msgs.Pose>() {
			@Override
			public void onNewMessage(geometry_msgs.Pose pose) {
				System.out.println("Received position update: ");
				phoenix.setPose(pose);
			}
		});
	}
}