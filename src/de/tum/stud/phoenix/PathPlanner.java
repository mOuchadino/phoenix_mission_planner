package de.tum.stud.phoenix;
import geometry_msgs.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
//global route planner computes paths ignoring the kinematic and dynamic vehicle constraints (not real-time)
//TODO: Implement D* Lite for replanning

public class PathPlanner {

	private MapNode[][] map;
	private int mapWidth, mapHeight;
	private LinkedList<MapNode> openList=new LinkedList<MapNode>();

	public PathPlanner(int[][] costMap) {
		mapWidth=costMap.length;
		mapHeight=costMap[0].length;
		map=new MapNode[mapHeight][mapWidth];
		for (int i = 0; i < mapHeight; i++) {//rows
			for (int j = 0; j < mapWidth; j++) {//cols
				map[i][j]=new MapNode();
				map[i][j].cost=costMap[j][i];
				map[i][j].g=costMap[j][i];
				map[i][j].obstacle=(costMap[j][i]>=2000)?true:false;
				map[i][j].nextObstacleDistance=(costMap[j][i]>=2000)?0:-1;
				map[i][j].visited=false;
				Point point =ApplicationContext.newInstance(Point.class, Point._TYPE);
				point.setX(j);
				point.setY(i);
				map[i][j].location=point;
			}
		}
		//bushfirePlanner();
	}

	public void bushfirePlanner(){
		for(int d=0;d<125;d++){
			for (int i = 0; i < mapHeight; i++) {//rows
				for (int j = 0; j < mapWidth; j++) {//cols
					if (map[i][j].nextObstacleDistance==d) //from the obstacles start a wave 
					{
						for(int r=-1;r<=1;r++){
							for(int c=-1;c<=1;c++){
								if(r==0 && c==0)//this is where we started;
								{
									continue; 
								}
								try{
									if(map[i+r][j+c].nextObstacleDistance==-1)// if until now undiscovered
									{
										map[i+r][j+c].nextObstacleDistance=d+1;
										map[i+r][j+c].waveOrigin=map[i][j];
									} else if (map[i+r][j+c].nextObstacleDistance==d) {//if two different wavefronts collide
										map[i+r][j+c].isGVD=true;
									}
								}
								catch(ArrayIndexOutOfBoundsException e){
								}
							}
						}
					}
				}
			}
		}	
	}

	public Path planPath(ArrayList<Point> viaPoints) throws MaxIterationsException {
		Path path=new Path();
		for(int i=0;i<viaPoints.size()-1;i++){//go through all segments between viapoints and plan
			System.out.println("Planning route segment "+(i+1));
			path.addSegment(planPathSegment(viaPoints.get(i), viaPoints.get(i+1)));
		}
		return path;
	}


	public ArrayList<Point> planPathSegment(Point startNode, Point goalNode) throws MaxIterationsException { //returns an arraylist of viapoints
		int iterations=0,maxIterations=200000;
		ArrayList<Point>routeSegment=new ArrayList<Point>();
		MapNode currentNode,neighbourNode;
		System.out.println("from ("+startNode.getX()+"/"+startNode.getY()+") to ("+goalNode.getX()+"/"+goalNode.getY()+")");

		map[(int)startNode.getY()][(int)startNode.getX()].parent=null;
		map[(int)startNode.getY()][(int)startNode.getX()].visited=false;
		map[(int)goalNode.getY()][(int)goalNode.getX()].cost=0;
		map[(int)goalNode.getY()][(int)goalNode.getX()].goal=true;
		openList.clear();
		openList.add(map[(int)startNode.getY()][(int)startNode.getX()]);//start at start node

		while(!openList.isEmpty() && iterations<maxIterations){//add neighbours to openlist and do goalcheck
			currentNode=openList.removeFirst();
			if(currentNode.goal==true)
			{   //we found it, return the path
				map[(int)goalNode.getY()][(int)goalNode.getX()].goal=false;
				openList.clear();
				return extractRoute(currentNode);
			} else {
				if (currentNode.location.getY()-1>=0 && currentNode.location.getY()-1<mapHeight)//above
				{
					for(int r=-1;r<=1;r++){
						for(int c=-1;c<=1;c++){
							if(r==0 && c==0)//this is where we started;
							{
								continue; 
							}
							try{
								neighbourNode=map[(int)currentNode.location.getY()+r][(int)currentNode.location.getX()+c];
								routeSegment=explore(currentNode, neighbourNode, goalNode);
								if(routeSegment != null){
									if(!routeSegment.isEmpty())
									{
										map[(int)goalNode.getY()][(int)goalNode.getX()].goal=false;
										openList.clear();
										return routeSegment;
									}
								}
							}
							catch(ArrayIndexOutOfBoundsException e){
							}
						}
					}

				}
				Collections.sort(openList);
				currentNode.visited=true;
				iterations++;
			}
		}
		if(iterations==maxIterations){
			throw new MaxIterationsException("Too many iterations in route planning");
		}
		return null;
	}

	public double h(Point from,Point to) {//euclidean distance as heuristic function
		return Math.sqrt(Math.pow((double)to.getX()-from.getX(),2)+Math.pow(to.getY()-from.getY(),2));
	}

	public ArrayList<Point> explore(MapNode currentNode, MapNode neighbourNode,Point to)
	{
		if(!neighbourNode.visited){
			if(neighbourNode.goal==true){//prematurely found goal
				neighbourNode.parent=currentNode;
				return extractRoute(neighbourNode);
			}
			neighbourNode.g=currentNode.g+neighbourNode.cost;
			neighbourNode.h=h(neighbourNode.location,to);
			neighbourNode.f=neighbourNode.g+neighbourNode.h;
			neighbourNode.parent=currentNode;
			if(!openList.contains(neighbourNode)){
				openList.add(neighbourNode);
			}
		}
		return null;
	}

	public ArrayList<Point> extractRoute(MapNode node)
	{
		ArrayList<Point> routeSegment=new ArrayList<Point>();
		while(node.parent!=null)
		{
			routeSegment.add(node.location);
			node=node.parent;
		}
		Collections.reverse(routeSegment);
		return routeSegment;
	}

	public MapNode[][] getNodeMap() {
		return map;
	}
}