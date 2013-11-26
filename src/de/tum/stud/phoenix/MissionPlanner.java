package de.tum.stud.phoenix;

import geometry_msgs.Point;

import java.awt.event.*;
import java.awt.BorderLayout;
import java.awt.Button;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Toolkit;

import javax.swing.BoxLayout;
import javax.swing.JCheckBoxMenuItem;
import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.Timer;
import javax.swing.WindowConstants;

public class MissionPlanner extends JFrame implements ItemListener {

	private static final long serialVersionUID = 1L;

	private Phoenix phoenix;
	private Trajectory trajectory;
	private MapReader mapReader;
	private PathPlanner pathPlanner;
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

	private Button planButton = new Button("1. Plan Route");
	private Button trajectoryButton = new Button("2. Generate Trajectory");
	private Button simButton = new Button("3. Simulate Flight");
	private Button flyButton = new Button("4. Fly");
	private Button resetButton = new Button("Reset");

	public MissionPlanner() { 
		super("Phoenix Mission Planner");
		
		try {
			org.ros.RosRun.main(new String[] { "de.tum.stud.phoenix.NavCmdPublisher" });
			navCmdPublisher = ApplicationContext.getNode(NavCmdPublisher.class);
		} catch (Exception e1) {
			System.out.println("Couldn't add NavCmdPublisher to ApplicationContext");
			e1.printStackTrace();
		}
		
		phoenix= new Phoenix();
		mapReader = new MapReader(mapName);
		
		//start the publisher and subscriber rosnodes

		try {
			org.ros.RosRun.main(new String[] { "de.tum.stud.phoenix.PoseListener" });
			poseListener = ApplicationContext.getNode(PoseListener.class);
			poseListener.setPhoenix(phoenix);
		} catch (Exception e1) {
			System.out.println("Couldn't add PoseListener to ApplicationContext");
			e1.printStackTrace();
		}

		trajectoryPlanner = new TrajectoryPlanner();
		pathPlanner = new PathPlanner(mapReader.getCostMap());
		mapPanel = new MapPanel(mapReader.getResizedMapImage(),pathPlanner.getNodeMap(), phoenix);

		//Menu
		JMenuBar menuBar;
		JMenu menu;
		JCheckBoxMenuItem cbMenuItem;
		menuBar = new JMenuBar();

		menu = new JMenu("Options");
		menu.setMnemonic(KeyEvent.VK_O);
		menuBar.add(menu);

		cbMenuItem = new JCheckBoxMenuItem("Show CostMap");
		cbMenuItem.setMnemonic(KeyEvent.VK_C);
		cbMenuItem.addItemListener(this);

		menu.add(cbMenuItem);
		this.setJMenuBar(menuBar);

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
			@Override
			public void mouseClicked(MouseEvent e) {
				Point currentPoint =ApplicationContext.newInstance(Point.class, Point._TYPE);
				currentPoint.setX(e.getPoint().x);
				currentPoint.setY(e.getPoint().y+25);

				if (mapPanel.getWayPoints().size()==0){ 
					phoenix.setLocation(currentPoint);
				}
				mapPanel.getWayPoints().add(currentPoint);
				mapPanel.repaint();
			}
		});
		cp.add(mapPanel, BorderLayout.CENTER);
		controlPanel= new JPanel();
		controlPanel.setSize(300, 100);
		controlPanel.setLayout(new BoxLayout(controlPanel, BoxLayout.X_AXIS));
		cp.add(controlPanel, BorderLayout.PAGE_END);  
		setVisible(true);

		planButton.addActionListener(new ActionListener() { 
			@Override
			public void actionPerformed(ActionEvent evt) { 
				plan_ActionPerformed(evt);}});
		controlPanel.add(planButton);

		trajectoryButton.addActionListener(new ActionListener() { 
			@Override
			public void actionPerformed(ActionEvent evt) { 
				trajectory_ActionPerformed(evt);}});
		controlPanel.add(trajectoryButton);

		simButton.addActionListener(new ActionListener() { 
			@Override
			public void actionPerformed(ActionEvent evt) { 
				sim_ActionPerformed(evt);}});
		controlPanel.add(simButton);
		
		flyButton.addActionListener(new ActionListener() { 
			@Override
			public void actionPerformed(ActionEvent evt) { 
				fly_ActionPerformed(evt);}});
		controlPanel.add(flyButton);

		resetButton.addActionListener(new ActionListener() { 
			@Override
			public void actionPerformed(ActionEvent evt) { 
				reset_ActionPerformed(evt);}});
		controlPanel.add(resetButton);

		repaint();
		setVisible(true);
	} 

	public static void main(String[] args) throws Exception {
		new MissionPlanner();
	}
	
	//menu item listener
	
	@Override
	public void itemStateChanged(ItemEvent e) {
		JMenuItem item=(JMenuItem) e.getSource();
		if (item.getText().equals("Show CostMap")) {
			if (e.getStateChange()==1) {
				mapPanel.enableCostMap();
			}else{
				mapPanel.disableCostMap();
			}	
		}
	}

	//plan the path
	public void plan_ActionPerformed(ActionEvent evt) {
		
		if (mapPanel.getWayPoints().size()<2) {
			System.out.println("Cannot plan path with less than start & goal");
			return;
		} 
		try {
			mapPanel.setPath(pathPlanner.planPath(mapPanel.getWayPoints()));
		} catch (MaxIterationsException e) {
			System.out.println("Too many iterations needed to plan path, maybe there is no path to this goal");
		}
		mapPanel.repaint();
	}

	//plan the trajectory
	public void trajectory_ActionPerformed(ActionEvent evt) {
		trajectory=trajectoryPlanner.plan(mapPanel.getPath());
		mapPanel.setTrajectory(trajectory);
		System.out.println("Generated trajectory");
		repaint();
	}

	//simulate the flight
	public void sim_ActionPerformed(ActionEvent evt) {
		System.out.println("Started simulation");
		navCmdPublisher.enableSimulation();
		flyFlight();
	}

	//fly
	public void fly_ActionPerformed(ActionEvent evt) {
		if(trajectory==null){
			System.out.println("Generate a trajectory and simulate first");
			return;
		}
		System.out.println("Started flight");
		navCmdPublisher.disableSimulation();
		navCmdPublisher.setTrajectory(trajectory);
		flyFlight();
	}
	
	//reset
	public void reset_ActionPerformed(ActionEvent evt) {
		mapPanel.getPath().clear();
		mapPanel.getWayPoints().clear();
		phoenix.resetGlobalYaw();
		repaint();
	}

	public void flyFlight(){
		int timerDelay = 50;
		if(trajectory==null){
			System.out.println("Generate a trajectory first");
			return;
		}
		ActionListener taskPerformer = new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent evt) {

				if(simStepCounter==trajectory.segments.size())//end of the simulation sequence
				{
					simTimer.stop();
					simStepCounter=0;
					phoenix.resetGlobalYaw();
					System.out.println("Finished ");
				}
				Point location= ApplicationContext.newInstance(Point.class, Point._TYPE);
				location.setX(trajectory.getPose(simStepCounter).getPosition().getX());
				location.setY(trajectory.getPose(simStepCounter).getPosition().getY());
				phoenix.setLocation(location);				
				repaint();
				simStepCounter++;
			}
		};
		simTimer=new Timer(timerDelay, taskPerformer);
		simTimer.start();
	}

	@Override
	public void paint(Graphics g) {
		super.paint(g); 
	}
} 