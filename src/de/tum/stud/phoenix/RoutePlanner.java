package de.tum.stud.phoenix;
import java.awt.Point;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
//global route planner computes paths ignoring the kinematic and dynamic vehicle constraints (not real-time)
//TODO: Implement D* Lite for replanning

public class RoutePlanner {

	private MapNode[][] map;
	private int mapWidth, mapHeight;
	private Route route=new Route();
	private LinkedList<MapNode> openList=new LinkedList<MapNode>();

	public RoutePlanner(int[][] costMap) {
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
				map[i][j].location=new Point(j,i);
			}
		}
		bushfirePlanner();
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

	public Route planRoute(ArrayList<Point> viaPoints) throws MaxIterationsException {
		if(viaPoints.size()<2)
		{
			System.out.println("Cannot plan route with less than start & goal ");
			return null;
		}else{
			for(int i=0;i<viaPoints.size()-1;i++){//go through all segments between viapoints and plan
				System.out.println("Planning route segment "+(i+1));
				route.addSegment(planRouteSegment(viaPoints.get(i), viaPoints.get(i+1)));
			}
		}
		return route;
	}


	public ArrayList<Point> planRouteSegment(Point from, Point to) throws MaxIterationsException { //gets an arraylist of viapoints
		int iterations=0,maxIterations=200000;
		ArrayList<Point>routeSegment=new ArrayList<Point>();
		Point startNode=from;
		Point goalNode=to;
		System.out.println("from ("+startNode.x+"/"+startNode.y+") to ("+goalNode.x+"/"+goalNode.y+")");
		map[startNode.y][startNode.x].parent=null;
		map[goalNode.y][goalNode.x].cost=0;
		map[goalNode.y][goalNode.x].goal=true;
		openList.add(map[startNode.y][startNode.x]);		//start at start node
		MapNode currentNode;
		MapNode neighbourNode;
		while(!openList.isEmpty() && iterations<maxIterations){//add neighbours to openlist and do goalcheck
			currentNode=openList.removeFirst();
			if(currentNode.goal==true)
			{//we found it, return the path
				map[goalNode.y][goalNode.x].goal=false;
				openList.clear();
				return extractRoute(currentNode);
			} else {
				if (currentNode.location.y-1>=0 && currentNode.location.y-1<mapHeight)//above
				{
					for(int r=-1;r<=1;r++){
						for(int c=-1;c<=1;c++){
							if(r==0 && c==0)//this is where we started;
							{
								continue; 
							}
							try{
								neighbourNode=map[currentNode.location.y+r][currentNode.location.x+c];
								routeSegment=explore(currentNode, neighbourNode, goalNode);
								if(routeSegment != null){
									if(!routeSegment.isEmpty())
									{
										map[goalNode.y][goalNode.x].goal=false;
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

	public double h(Point from,Point to) {
		return Math.sqrt(Math.pow((double)to.x-from.x,2)+Math.pow(to.y-from.y,2));
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

	public Route getRoute() {
		return route;
	}

	public void setRoute(Route route) {
		this.route = route;
	}

	public MapNode[][] getNodeMap() {
		return map;
	}
}