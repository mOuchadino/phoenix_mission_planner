import java.awt.Point;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;

public class RoutePlanner {

	private Point startNode;
	private Point goalNode;
	private Node[][] map;
	private ArrayList<Point> route=new ArrayList<Point>();
	private LinkedList<Node> openList=new LinkedList<Node>();
	private int mapWidth, mapHeight;

	public RoutePlanner(int[][] costMap) {
		mapWidth=costMap.length;
		mapHeight=costMap[0].length;
		map=new Node[mapHeight][mapWidth];
		for (int i = 0; i < mapHeight; i++) {//rows
			for (int j = 0; j < mapWidth; j++) {//cols
				map[i][j]=new Node();
				map[i][j].cost=costMap[j][i];
				map[i][j].g=costMap[j][i];
				map[i][j].visited=false;
				map[i][j].location=new Point(j,i);
			}
		}
	}

	public void setPosition(Point currentPosition) {
		startNode=currentPosition;
	}

	public void setGoal(Point goalPosition) {
		goalNode=goalPosition;		
	}

	public ArrayList<Point> planRoute(Point from, Point to) {
		startNode=from;
		goalNode=to;
		System.out.println("Map Dimensions "+mapWidth+"x"+mapHeight+")");
		System.out.println("Start is at ("+startNode.x+"/"+startNode.y+")");
		System.out.println("Goal is at ("+goalNode.x+"/"+goalNode.y+")");
		map[goalNode.y][goalNode.x].cost=0;
		map[goalNode.y][goalNode.x].goal=true;
		openList.add(map[startNode.y][startNode.x]);		//start at start node
		Node currentNode;
		Node neighbourNode;
		while(!openList.isEmpty()){//add neighbours to openlist and do goalcheck
			currentNode=openList.removeFirst();
			if(currentNode.goal==true)
			{//we found it, return the path
				System.out.println("Found goal at"+currentNode.location);
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
								explore(currentNode, neighbourNode);
								if(!route.isEmpty())
								{
									return route;
								}
							}
							catch(ArrayIndexOutOfBoundsException e){
							}
						}
					}

				}
				Collections.sort(openList);
				currentNode.visited=true;
			}
		}

		return null;
	}

	public double h(Point from) {
		return Math.sqrt(Math.pow((double)goalNode.x-from.x,2)+Math.pow(goalNode.y-from.y,2));
	}

	public void  explore(Node currentNode, Node neighbourNode)
	{
		if(!neighbourNode.visited){
			if(neighbourNode.goal==true){
				System.out.println("Found goal at"+neighbourNode.location);
				neighbourNode.parent=currentNode;
				route=extractRoute(neighbourNode);
			}
			neighbourNode.g=currentNode.g+neighbourNode.cost;
			neighbourNode.h=h(neighbourNode.location);
			neighbourNode.f=neighbourNode.g+neighbourNode.h;
			neighbourNode.parent=currentNode;
			if(!openList.contains(neighbourNode)){
				openList.add(neighbourNode);
			}
		}
	}

	public ArrayList<Point> extractRoute(Node node)
	{
		while(node.parent!=null)
		{
			route.add(node.location);
			node=node.parent;
		}
		Collections.reverse(route);

		for(Point p:route)
		{
			System.out.println(p);
		}
		return route;
	}

	public ArrayList<Point> getRoute() {
		return route;
	}

	public void setRoute(ArrayList<Point> route) {
		this.route = route;
	}
	
	class Node implements Comparable<Node>
	{
		Point location;
		int cost;
		int g;
		double h;
		double f;
		boolean goal;
		boolean visited;
		Node parent;

		@Override
		public int compareTo(Node compareObject)
		{
			if (f < compareObject.f)
				return -1;
			else if (f == compareObject.f)
				return 0;
			else
				return 1;
		}
	}
}

