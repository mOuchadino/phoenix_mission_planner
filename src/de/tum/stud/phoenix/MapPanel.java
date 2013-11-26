package de.tum.stud.phoenix;
import geometry_msgs.Point;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
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
	private BufferedImage resizedImage,phoenixImg, goalImg, flagImg;
	private MapNode[][] map;
	private int mapWidth, mapHeight;
	private ArrayList<Point> wayPoints = new ArrayList<Point>();
	private Trajectory trajectory;
	private Path path = new Path();
	private Phoenix phoenix;
	private boolean drawCostMap=false;


	public void enableCostMap() {
		this.drawCostMap = true;
		this.repaint();
		System.out.println("Enabled Costmap");
	}

	public void disableCostMap() {
		this.drawCostMap = false;
		this.repaint();
		System.out.println("Disabled Costmap");
	}

	public MapPanel(BufferedImage resizedImage,MapNode[][] map, Phoenix phoenix) {               
		this.resizedImage=resizedImage;
		this.map=map;
		mapHeight=this.map.length;
		mapWidth=this.map[0].length;
		this.phoenix=phoenix;
		try{
			phoenixImg = ImageIO.read(new File("img/blimp.png"));
			goalImg = ImageIO.read(new File("img/goal.png"));
			flagImg = ImageIO.read(new File("img/waypoint.png"));

		}catch(IOException e){
			System.out.println(e.getMessage());
		}
	}

	public Path getPath() {
		return path;
	}

	public void setPath(Path path) {
		this.path = path;
	}

	public ArrayList<Point> getWayPoints() {
		return wayPoints;
	}

	public void setWayPoints(ArrayList<Point> wayPoints) {
		this.wayPoints = wayPoints;
	}

	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		Graphics2D g2d = (Graphics2D)g;

		//draw the map as image
		g2d.drawImage(resizedImage, 0, 0, null);

		//overlay the costMap
		if (drawCostMap) {
			for(int i = 0; i < mapWidth; i++) {
				for(int j = 0; j < mapHeight; j++) {
					float grey = (2001 - map[j][i].cost) / 2001f;
					g2d.setColor(new Color(grey,grey,grey));
					g2d.drawLine(i,j,i,j);  
				}
			}
		}

		//overlay bushfire planner
		/*
		for(int i = 0; i < mapWidth; i++) {
			for(int j = 0; j < mapHeight; j++) {
				if (map[j][i].nextObstacleDistance==0 ) {
					continue;
				}else if (map[j][i].nextObstacleDistance==-1 ) {
					continue;
				} else {
					g2d.setColor(new Color(map[j][i].nextObstacleDistance*2,0,0));
				}
				if(map[j][i].isGVD==true)
					g2d.setColor(Color.ORANGE);
				g2d.drawLine(i,j,i,j);  
			}
		}*/


		BasicStroke stroke3= new BasicStroke(2.5f, BasicStroke.CAP_SQUARE, BasicStroke.JOIN_BEVEL);
		g2d.setStroke(stroke3);

		//draw path in red
		if(path!=null){
			g2d.setColor(new Color(255, 0,0));
			for(ArrayList<Point> segment:path.getSegments()){
				for (int i=0; i<segment.size()-2; i++ ) {
					g2d.drawLine((int)segment.get(i).getX(),(int)segment.get(i).getY(),(int)segment.get(i+1).getX(),(int)segment.get(i+1).getY());  
				} 
			} 
		}

		//draw trajectory in blue
		if(trajectory!=null){
			g2d.setColor(new Color(0, 0, 255));
			for(ArrayList<Point> segment:path.getSegments()){
					for (int i=0; i<segment.size()-1; i++ ) {
						g2d.drawLine((int)segment.get(i).getX(),(int)segment.get(i).getY(),(int)segment.get(i+1).getX(),(int)segment.get(i+1).getY());  
					}
			}
		}


		//draw original bee-line path in black
		g2d.setColor(new Color(0, 0, 0));
		for (int i=0; i<wayPoints.size()-1; i++ ) {
			g2d.drawLine((int)wayPoints.get(i).getX(),(int)wayPoints.get(i).getY(),(int)wayPoints.get(i+1).getX(),(int)wayPoints.get(i+1).getY());
		}

		// draw waypoints and goal
		for (Point p : wayPoints){
			if(wayPoints.indexOf(p)==0)
			{
			} else if (wayPoints.indexOf(p)==wayPoints.size()-1){//draw goal
				g2d.drawImage(goalImg, (int)p.getX()-25, (int)p.getY()-40, null);
			} else {//draw via point
				g2d.drawImage(flagImg, (int)p.getX()-16, (int)p.getY()-30, null);
			}
		}

		//calculate the transform of the phoenix

		double globalYaw = Math.asin(2*phoenix.getPose().getOrientation().getX()*phoenix.getPose().getOrientation().getY() + 2*phoenix.getPose().getOrientation().getZ()*phoenix.getPose().getOrientation().getW());
		double centerX = phoenixImg.getWidth()/2;
		double centerY = phoenixImg.getHeight()/2;
		AffineTransform tx = AffineTransform.getRotateInstance(globalYaw, centerX, centerY);
		AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
		// Drawing the rotated image at the required drawing locations
		g2d.drawImage(op.filter(phoenixImg, null), ((int)phoenix.getPose().getPosition().getX())-35, ((int)phoenix.getPose().getPosition().getY())-35, null);
	}

	public Trajectory getTrajectory() {
		return trajectory;
	}

	public void setTrajectory(Trajectory trajectory) {
		this.trajectory = trajectory;
	}
}