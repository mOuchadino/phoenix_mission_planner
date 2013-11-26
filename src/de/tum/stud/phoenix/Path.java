package de.tum.stud.phoenix;
import geometry_msgs.Point;

import java.util.ArrayList;

//a path can consist of several path segments (between the viapoints)
//a pathsegment is a list of points

public class Path{
	private ArrayList<ArrayList<Point>> segments = new ArrayList<ArrayList<Point>>();

	public ArrayList<ArrayList<Point>> getSegments() {
		if (segments==null) {
			System.out.println("Warning, path is null");
		}
		return segments;
	}

	public void clear() {
		segments.clear();
	}
	
	public void addSegment(ArrayList<Point> segment) {
		segments.add(segment);
	}
}