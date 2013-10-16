package de.tum.stud.phoenix;
import java.awt.Point;
import java.util.ArrayList;

//a route consists of several route segments (between the viapoints)

public class Route implements Cloneable {
	private ArrayList<ArrayList<Point>> segments = new ArrayList<ArrayList<Point>>();

	public ArrayList<ArrayList<Point>> getSegments() {
		return segments;
	}

	public void clear() {
		segments.clear();
	}
	
	 public Route clone() {
		 Route newRoute=new Route();
		 for(ArrayList<Point> segment:segments)
		 {
			 ArrayList<Point> newSegment=new ArrayList<Point>();
			 for(Point point:segment)
			 {
				 Point newPoint=new Point();
				 newPoint=(Point) point.clone();
				 newSegment.add(newPoint);
			 }
			 newRoute.addSegment(newSegment);
		 }
		return newRoute;
	}

	public void addSegment(ArrayList<Point> segment) {
		segments.add(segment);
	}
}