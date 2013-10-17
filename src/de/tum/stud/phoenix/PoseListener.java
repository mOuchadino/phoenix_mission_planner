package de.tum.stud.phoenix;
import java.awt.geom.Point2D;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

/**
listens for new pose updates on /navdata in geometry_msgs/Twist form and applies them to the model.
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
		final Log log = connectedNode.getLog();
		Subscriber<geometry_msgs.Twist> subscriber = connectedNode.newSubscriber("navdata", geometry_msgs.Twist._TYPE);
		subscriber.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
			@Override
			public void onNewMessage(geometry_msgs.Twist twist) {
		        log.info("I heard: \"" + twist.toString() + "\"");

				System.out.println("Got new position");
				phoenix.setLocation(new Point2D.Double(twist.getLinear().getX(),twist.getLinear().getY()));
				phoenix.setYaw(twist.getAngular().getZ());
			}
		});
	}
}