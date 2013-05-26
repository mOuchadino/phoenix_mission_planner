import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import javax.imageio.ImageIO;
import javax.swing.JPanel;


public class MapPanel extends JPanel{

	private static final long serialVersionUID = -6348356140496920472L;
	private BufferedImage resizedImage;
	private int[][] costMap;

	private ArrayList<Point> wayPoints = new ArrayList<Point>();
	private ArrayList<Point> wayPointsSmoothed= new ArrayList<Point>();
	private Route route = new Route();
	private Phoenix phoenix;
	public boolean drawRoute=false;

	BufferedImage phoenixImg, goalImg, flagImg;


	public MapPanel(BufferedImage map,int[][] costMap, Phoenix phoenix) {               
		this.resizedImage=map;
		this.costMap=costMap;
		this.phoenix=phoenix;
		try{
			phoenixImg = ImageIO.read(new File("img/blimp.png"));
			goalImg = ImageIO.read(new File("img/goal.png"));
			flagImg = ImageIO.read(new File("img/waypoint.png"));

		}catch(IOException e){
			System.out.println(e.getMessage());
		}
	}

	public Route getRoute() {
		return route;
	}

	public void setRoute(Route route) {
		this.route = route;
		drawRoute=true;
	}

	public ArrayList<Point> getWayPoints() {
		return wayPoints;
	}

	public void setWayPoints(ArrayList<Point> wayPoints) {
		this.wayPoints = wayPoints;
	}

	public ArrayList<Point> getWayPointsSmoothed() {
		return wayPointsSmoothed;
	}

	public void setTrajectory(ArrayList<Point> wayPointsSmoothed) {
		this.wayPointsSmoothed = wayPointsSmoothed;
	}

	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		Graphics2D g2d = (Graphics2D)g;

		//draw the map as image
		g2d.drawImage(resizedImage, 0, 0, null);

		//overlay the costMap
		for(int i = 0; i < 1024; i++) {
			for(int j = 0; j < 533; j++) {
				if (costMap[i][j]>=2000) {
					//g2d.setColor(new Color(0,0,0));
					continue;
				}else if(costMap[i][j]>=14) {
					g2d.setColor(new Color(0.2f, 0.2f,0.2f,0.2f));
				}else if(costMap[i][j]>=1){
					g2d.setColor(new Color(255,255,255));
				}
				g2d.drawLine(i,j,i,j);  
			}
		}


		BasicStroke stroke3= new BasicStroke(2.5f, BasicStroke.CAP_SQUARE, BasicStroke.JOIN_BEVEL);
		g2d.setStroke(stroke3);

		//draw route in red
		if(drawRoute){
			g2d.setColor(new Color(255, 0,0));
			for(ArrayList<Point> segment:route.getSegments()){
				for (int i=0; i<segment.size()-2; i++ ) {
					g2d.drawLine(segment.get(i).x,segment.get(i).y,segment.get(i+1).x,segment.get(i+1).y);  
				} 
			} 
		}

		//draw smoothed path in blue
		g2d.setColor(new Color(0, 0, 255));
		for (int i=0; i<wayPointsSmoothed.size()-1; i++ ) {
			g2d.drawLine(wayPointsSmoothed.get(i).x,wayPointsSmoothed.get(i).y,wayPointsSmoothed.get(i+1).x,wayPointsSmoothed.get(i+1).y);  
		} 

		//draw original path in black
		g2d.setColor(new Color(0, 0, 0));
		for (int i=0; i<wayPoints.size()-1; i++ ) {
			g2d.drawLine(wayPoints.get(i).x,wayPoints.get(i).y,wayPoints.get(i+1).x,wayPoints.get(i+1).y);
		}

		// draw waypoints and goal
		for (Point p : wayPoints){
			if(wayPoints.indexOf(p)==0)
			{
			} else if (wayPoints.indexOf(p)==wayPoints.size()-1){//draw goal
				g2d.drawImage(goalImg, p.x-25, p.y-40, null);
			} else {//draw via point
				g2d.drawImage(flagImg, p.x-16, p.y-30, null);
			}
		}

		//calculate the transform of the phoenix
		double angle = Math.toRadians(phoenix.getGlobalYaw());
		double centerX = phoenixImg.getWidth() / 2;
		double centerY = phoenixImg.getHeight() / 2;
		AffineTransform tx = AffineTransform.getRotateInstance(angle, centerX, centerY);
		AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
		// Drawing the rotated image at the required drawing locations
		g2d.drawImage(op.filter(phoenixImg, null), phoenix.getLocation().x-35, phoenix.getLocation().y-35, null);
	}
}