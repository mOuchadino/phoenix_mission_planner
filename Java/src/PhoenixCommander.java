import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import javax.imageio.ImageIO;
import javax.swing.*;
import java.util.ArrayList;
import java.awt.AlphaComposite;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.RenderingHints;
import java.io.File;
import java.io.IOException;

public class PhoenixCommander extends JFrame {
  
  private static final long serialVersionUID = 1L;
  private String mapName="img/floorplan.png";
  BufferedImage map, resizedMap;
  private double scaleFactor=0.1;
  private MapReader mapReader;
  private RoutePlanner routePlanner;
  private TrajectoryPlanner trajectoryPlanner;
  private int windowWidth;
  private int windowHeight;
  private MapPanel mapPanel;
  private JPanel controlPanel;
  private Point phoenixPosition;
  private ArrayList<Point> wayPoints = new ArrayList<Point>();
  private ArrayList<Point> wayPointsSmoothed= new ArrayList<Point>();
  private Button bu4 = new Button("1. Plan Route");
  private Button bu2 = new Button("2. Smoothe Trajectory");
  private Button bu3 = new Button("3. Simulate Flight");
  private Button bu1 = new Button("Reset");  
  
  public PhoenixCommander() { 
    super("PhoenixCommander");
    
    try{
  	  map = ImageIO.read(new File(mapName));
  	  resizedMap = resizeImageWithHint(map, map.getType(), (int)(map.getWidth()*scaleFactor), (int)(map.getHeight()*scaleFactor));
  	  mapReader = new MapReader(resizedMap);
  	  routePlanner = new RoutePlanner(mapReader.getMap());
  	  trajectoryPlanner = new TrajectoryPlanner();
      mapPanel = new MapPanel(resizedMap);
      windowWidth=resizedMap.getWidth();
      windowHeight=resizedMap.getHeight()+50;
  	}catch(IOException e){
  	  System.out.println(e.getMessage());
  	}
    
    setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    setSize(windowWidth, windowHeight);
    Dimension d = Toolkit.getDefaultToolkit().getScreenSize();
    setLocation((d.width-getSize().width)/2,(d.height-getSize().height)/2);
    setResizable(false);
    Container cp = getContentPane();
    cp.setLayout(new BorderLayout(5,5));
    mapPanel.addMouseListener(new MouseAdapter() {
      public void mouseClicked(MouseEvent e) {
        Point currentPoint = new Point (e.getPoint().x, e.getPoint().y+25);
        
        if (wayPoints.size()>0){ 
        	Point lastPoint = wayPointsSmoothed.get(wayPointsSmoothed.size()-1);
        	int distance = (int)Math.sqrt(((lastPoint.x-currentPoint.x)*(lastPoint.x-currentPoint.x))+
        			((lastPoint.y-currentPoint.y)*(lastPoint.y-currentPoint.y)));
        	System.out.println("Distanz:  "+ distance);
        	int wieviel = ((int)distance/15)-2;
        	System.out.println("Wieviele:  "+ wieviel);
          
        	if (wieviel >= 1) {
        		for (int i=1; i<wieviel; i++ ) {
        			wayPointsSmoothed.add(new Point((int)(lastPoint.x + i * ((currentPoint.x-lastPoint.x)*1.0/wieviel)),
        					(int)(lastPoint.y + i * ((currentPoint.y-lastPoint.y)*1.0/wieviel))));
        		} 
        	} 
        	wayPointsSmoothed.add((Point)currentPoint.clone());
        	wayPoints.add((Point)currentPoint.clone());
        	routePlanner.setGoal((Point)currentPoint.clone());

        } else { //handle blimp position & first waypoint added
        	phoenixPosition=(Point)currentPoint.clone();
        	routePlanner.setPosition((Point)currentPoint.clone());
        	paintPhoenix();
        	wayPoints.add((Point)currentPoint.clone());
        	wayPointsSmoothed.add((Point)currentPoint.clone());  
        } 
        renderWaypoints();
      }
    });
    mapPanel.setBackground(Color.cyan);
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
  
  public static void main(String[] args) {
    new PhoenixCommander();
  } 
    
  public void renderWaypoints(){
	mapPanel.removeAll();
    for (Point p : wayPoints){
    	if(wayPoints.indexOf(p)==0)
    	{
    		paintPhoenix();
    	} else if (wayPoints.indexOf(p)==wayPoints.size()-1){
    		paintGoal(p);
    		routePlanner.setGoal(p);
    	} else {
    		paintFlag(p);
    	}
    }
    repaint();
  }
  
  public void paintFlag(Point p){
	    ImageIcon flagIcon = new ImageIcon("img/waypoint.png");
	      JLabel label= new JLabel(flagIcon);
	      label.setSize(32,32);
	      label.setLocation(p.x-16,p.y-56);
	      mapPanel.add(label);
	      mapPanel.repaint();
	  }
  
  public void paintPhoenix(){
	    ImageIcon phoenixIcon = new ImageIcon("img/blimp.png");
	      JLabel label= new JLabel(phoenixIcon);
	      label.setSize(70,27);
	      label.setLocation(phoenixPosition.x-35,phoenixPosition.y-35);
	      mapPanel.add(label);
	      mapPanel.repaint();
	  }
  
  public void paintGoal(Point p){
	    ImageIcon goalIcon = new ImageIcon("img/goal.png");
	      JLabel label= new JLabel(goalIcon);
	      label.setSize(50,50);
	      label.setLocation(p.x-25,p.y-50);
	      mapPanel.add(label);
	      mapPanel.repaint();
	  }
  
  public void bu1_ActionPerformed(ActionEvent evt) {
	  	wayPoints.clear();
	  	wayPointsSmoothed.clear();
	  	mapPanel.removeAll();
    	repaint();
  }
  
  @SuppressWarnings("unchecked")
public void bu2_ActionPerformed(ActionEvent evt) {
	  	wayPointsSmoothed=trajectoryPlanner.smootheTrajectory( (ArrayList<Point>) wayPointsSmoothed.clone());
	  	trajectoryPlanner.printresult("all");
    	renderWaypoints();
    	repaint();
  }
  
  public void bu3_ActionPerformed(ActionEvent evt) {
	  	simulateFlight();
}
  
  public void bu4_ActionPerformed(ActionEvent evt) {
	  	wayPoints=routePlanner.planRoute();
	  	renderWaypoints();
	  	repaint();
}
  
  private void simulateFlight() {
	// TODO Auto-generated method stub
}

public void paint(Graphics g) {
	  super.paint(g); 
	  Graphics2D g2d = (Graphics2D)g;
    
	  BasicStroke stroke3= new BasicStroke(2.5f, BasicStroke.CAP_SQUARE, BasicStroke.JOIN_BEVEL);
	  g2d.setStroke(stroke3);
    
	  //draw smoothed path in blue
	  g.setColor(new Color(0, 0, 255));
	  for (int i=0; i<wayPointsSmoothed.size()-1; i++ ) {
		  g.drawLine(wayPointsSmoothed.get(i).x,wayPointsSmoothed.get(i).y,wayPointsSmoothed.get(i+1).x,wayPointsSmoothed.get(i+1).y);  
	  } 
	  
	  //draw original path in black
	  g.setColor(new Color(0, 0, 0));
	  for (int i=0; i<wayPoints.size()-1; i++ ) {
		  g.drawLine(wayPoints.get(i).x,wayPoints.get(i).y,wayPoints.get(i+1).x,wayPoints.get(i+1).y);    
	  }	      
  }

private BufferedImage resizeImageWithHint(BufferedImage originalImage, int type, int width, int height){
	 BufferedImage resizedImage = new BufferedImage(width, height, type);
	 Graphics2D g = resizedImage.createGraphics();
	 g.drawImage(originalImage, 0, 0, width, height, null);
	 g.dispose();	
	 g.setComposite(AlphaComposite.Src);

	 g.setRenderingHint(RenderingHints.KEY_INTERPOLATION,RenderingHints.VALUE_INTERPOLATION_BILINEAR);
	 g.setRenderingHint(RenderingHints.KEY_RENDERING,RenderingHints.VALUE_RENDER_QUALITY);
	 g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON);

return resizedImage;
}
} 