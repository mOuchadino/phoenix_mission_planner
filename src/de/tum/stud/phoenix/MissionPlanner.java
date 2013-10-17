package de.tum.stud.phoenix;

import java.awt.*;
import java.awt.event.*;
import java.util.ArrayList;
import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Graphics;
import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.Timer;
import javax.swing.WindowConstants;

public class MissionPlanner extends JFrame {

	private static final long serialVersionUID = 1L;

	private Phoenix phoenix;
	private MapReader mapReader;
	private RoutePlanner routePlanner;
	private TrajectoryPlanner trajectoryPlanner;
	private PoseListener poseListener;
	private NavCmdPublisher navCmdPublisher;
	private String mapName="img/floorplan.png";
	private int windowWidth;
	private int windowHeight;
	private int simStepCounter=0;
	private Timer simTimer;
	private MapPanel mapPanel;
	private JPanel controlPanel;
	private ArrayList<Float> angles= new ArrayList<Float>();
	private Button bu4 = new Button("1. Plan Route");
	private Button bu2 = new Button("2. Generate Trajectory");
	private Button bu3 = new Button("3. Simulate Flight");
	private Button bu1 = new Button("Reset");

	public MissionPlanner() { 
		super("Phoenix Mission Planner");
		phoenix= new Phoenix();
		mapReader = new MapReader(mapName);
		try {
			org.ros.RosRun.main(new String[] { "de.tum.stud.phoenix.Listener" });
		} catch (Exception e3) {
			// TODO Auto-generated catch block
			e3.printStackTrace();
		}
		try {
			org.ros.RosRun.main(new String[] { "de.tum.stud.phoenix.Talker" });
		} catch (Exception e2) {
			// TODO Auto-generated catch block
			e2.printStackTrace();
		}

		try {
			org.ros.RosRun.main(new String[] { "de.tum.stud.phoenix.NavCmdPublisher" });
			navCmdPublisher = ApplicationContext.getNode(NavCmdPublisher.class);
			navCmdPublisher.setPhoenix(phoenix);
		} catch (Exception e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		try {
			org.ros.RosRun.main(new String[] { "de.tum.stud.phoenix.PoseListener" });
			poseListener = ApplicationContext.getNode(PoseListener.class);
			//poseListener.setPhoenix(phoenix);
		} catch (Exception e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		trajectoryPlanner = new TrajectoryPlanner();
		routePlanner = new RoutePlanner(mapReader.getCostMap());
		mapPanel = new MapPanel(mapReader.getResizedMapImage(),routePlanner.getNodeMap(), phoenix);
		windowWidth=mapReader.getResizedMapImage().getWidth();
		windowHeight=mapReader.getResizedMapImage().getHeight()+50;   
		setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
		setSize(windowWidth, windowHeight);
		Dimension d = Toolkit.getDefaultToolkit().getScreenSize();
		setLocation((d.width-getSize().width)/2,(d.height-getSize().height)/2);
		setResizable(true);
		Container cp = getContentPane();
		cp.setLayout(new BorderLayout(5,5));
		mapPanel.addMouseListener(new MouseAdapter() {
			public void mouseClicked(MouseEvent e) {
				Point currentPoint = new Point (e.getPoint().x, e.getPoint().y+25);
				if (mapPanel.getWayPoints().size()>0){ 
					Point lastPoint = mapPanel.getWayPointsSmoothed().get(mapPanel.getWayPointsSmoothed().size()-1);
					int distance = (int)Math.sqrt(((lastPoint.x-currentPoint.x)*(lastPoint.x-currentPoint.x))+
							((lastPoint.y-currentPoint.y)*(lastPoint.y-currentPoint.y)));
					System.out.println("Distanz:  "+ distance);
					int wieviel = ((int)distance/15)-2;
					System.out.println("Wieviele:  "+ wieviel);

					if (wieviel >= 1) {
						for (int i=1; i<wieviel; i++ ) {
							mapPanel.getWayPointsSmoothed().add(new Point((int)(lastPoint.x + i * ((currentPoint.x-lastPoint.x)*1.0/wieviel)),
									(int)(lastPoint.y + i * ((currentPoint.y-lastPoint.y)*1.0/wieviel))));
						} 
					} 
					mapPanel.getWayPointsSmoothed().add((Point)currentPoint.clone());
					mapPanel.getWayPoints().add((Point)currentPoint.clone());

				} else { //handle blimp position & first waypoint added
					phoenix.setLocation(currentPoint);
					mapPanel.getWayPoints().add((Point)currentPoint.clone());
					mapPanel.getWayPointsSmoothed().add((Point)currentPoint.clone());  
				}
				mapPanel.repaint();
			}
		});
		cp.add(mapPanel, BorderLayout.CENTER);
		controlPanel= new JPanel();
		controlPanel.setSize(300, 100);
		controlPanel.setLayout(new BoxLayout(controlPanel, BoxLayout.X_AXIS));
		cp.add(controlPanel, BorderLayout.PAGE_END);  
		setVisible(true);

		bu4.addActionListener(new ActionListener() { 
			public void actionPerformed(ActionEvent evt) { 
				bu4_ActionPerformed(evt);}});
		controlPanel.add(bu4);

		bu2.addActionListener(new ActionListener() { 
			public void actionPerformed(ActionEvent evt) { 
				bu2_ActionPerformed(evt);}});
		controlPanel.add(bu2);

		bu3.addActionListener(new ActionListener() { 
			public void actionPerformed(ActionEvent evt) { 
				bu3_ActionPerformed(evt);}});
		controlPanel.add(bu3);

		bu1.addActionListener(new ActionListener() { 
			public void actionPerformed(ActionEvent evt) { 
				bu1_ActionPerformed(evt);}});
		controlPanel.add(bu1);

		repaint();
		setVisible(true);
	} 

	/**
	 * 
	 * @param args
	 * @throws Exception
	 */
	public static void main(String[] args) throws Exception {
		new MissionPlanner();
	} 

	public void bu1_ActionPerformed(ActionEvent evt) {
		mapPanel.getRoute().clear();
		mapPanel.getWayPoints().clear();
		mapPanel.getWayPointsSmoothed().clear();
		routePlanner.getRoute().clear();
		phoenix.resetGlobalYaw();
		repaint();
	}

	//find the path
	public void bu4_ActionPerformed(ActionEvent evt) {
		try {
			mapPanel.setRoute(routePlanner.planRoute(mapPanel.getWayPoints()));
		} catch (MaxIterationsException e) {
			System.out.println("Too many iterations needed to plan route, maybe no route to point");
		}
		mapPanel.repaint();
	}

	//smoothe the trajectory
	public void bu2_ActionPerformed(ActionEvent evt) {
		mapPanel.setTrajectory(trajectoryPlanner.smootheTrajectory( mapPanel.getRoute().clone()));
		repaint();
	}

	//simulate the flight
	public void bu3_ActionPerformed(ActionEvent evt) {
		simulateFlight();
	}

	public void simulateFlight(){
		int timerDelay = 50;
		angles=trajectoryPlanner.calculateAngles(mapPanel.getWayPointsSmoothed());
		ActionListener taskPerformer = new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				if(simStepCounter==mapPanel.getWayPointsSmoothed().size()-1)
				{
					simTimer.stop();
					simStepCounter=0;
					phoenix.resetGlobalYaw();
				}
				if (angles.get(simStepCounter)!=0.0){
					phoenix.setYaw(angles.get(simStepCounter));
				}
				phoenix.setLocation(mapPanel.getWayPointsSmoothed().get(simStepCounter));
				repaint();
				simStepCounter++;
			}
		};

		simTimer=new Timer(timerDelay, taskPerformer);
		simTimer.start();;
	}

	public void paint(Graphics g) {
		super.paint(g); 
	}
} 