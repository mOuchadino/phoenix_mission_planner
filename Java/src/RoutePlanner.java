import java.awt.Point;
import java.util.ArrayList;

public class RoutePlanner {
	
    private Point startNode;
    private Point goalNode;
    private  int[][] map={{1,1,1,1,1}, {1,100,1,1,1}, {1,1,1,1,1}, {1,1,1,1,1}, {1,1,1,1,1}};
    private ArrayList<Point> route;
    
    public RoutePlanner(int[][] initMap) {
    	map=initMap;
    }
	
	public void setPosition(Point currentPosition) {
		startNode=currentPosition;
	}

	public void setGoal(Point goalPosition) {
		goalNode=goalPosition;		
	}
	
	public void testRouting(){
		startNode=new Point(1,5);
		goalNode=new Point(24,20);
		System.out.println("Start is at ("+startNode.x+"/"+startNode.y+")");
		System.out.println("Goal is at ("+goalNode.x+"/"+goalNode.y+")");
		printMap();
	    route = planRoute();
	    for (Point p : route) {
            System.out.print(p + "\t");
        }
	}
	
	public void printMap(){
		//Show the array on the display:
	    for (int k = 0; k < map.length; ++k) {
	        for (int l = 0; l < map[0].length; ++l) {
	            System.out.print(map[k][l] + "\t");
	        }
	        System.out.println("");	  
	    }
	}
		
	
	//The path finding method
	//Takes as arguments the first node s, the goal node g 
	//Returns the shortest way in form of an int-array
	
	public ArrayList<Point> planRoute() {

		//Initialization:
		int w = map[0].length; //width of the map (in bit)
		int h = map.length;  //height of the map (in bit)
		
		boolean arrived = false;
		int costAct = 0;  //actual costs
		int[] openList = new int[h*w];
		int[] way = new int[h*w];
		boolean[] closed = new boolean[h*w];  //closed list
		boolean[] open = new boolean[h*w];  //open list marker
		int openListCounter = 1;
		int[] openListG = new int[h*w]; //G-value of A*
		int[] openListH = new int[h*w]; //H-value of A*
		int[] openListF = new int[h*w]; //F-value of A*
		int[] parent = new int[h*w];
		for(int i = 0; i < h*w; i++) {
			closed[i] = false;
			open[i] = false;
			way[i] = -1;
			openList[i] = -1;
			openListH[i] = calculateh(goalNode, i, w, h);
			parent[i] = -2;
		}
		
		int nextNode;
		int b, savei = 0;

		//insert the start node into the open list:
		System.out.println(h*w);
		open[startNode] = true;
		openList[0] = startNode;
		openListG[startNode] = 0;
		openListF[startNode] = openListH[startNode];
		nextNode = startNode;
		boolean check = false; //for choosing node from openList

		//Attention: no solution if their is NO way!
		do {
			//find the node in open list with lowest f-value:
			for(int i = 0; i < h*w; i++) {
				if (openList[i] > -1) {
					if (check) {
						nextNode = openList[i];
						check = false;
						savei = i;
					}
					if (openListF[openList[i]] < openListF[nextNode]){
						nextNode = openList[i];
						savei = i;
					}
				}
			}
			check = true;
			if (nextNode == goalNode){
				costAct = openListG[goalNode];
				arrived = true;
			}
			else {
				closed[nextNode] = true;
				open[nextNode] = false;
				openList[savei] = -1;
				costAct = openListG[nextNode];
				//Consider the neighbors of the actual node:
				int kh = nextNode/w;
				int kw = nextNode - Math.max(0, kh)*w;

				
				for(int ah = Math.max(0, kh-1); ah <= Math.min(h-1, kh+1); ah++) {
					for(int aw = Math.max(0, kw-1); aw <= Math.min(w-1, kw+1); aw++) {
						
						b = w*Math.max(0, ah) + aw;
						
						//if node is in the open list and costAct is lower than it's g
						if (open[b] && (openListG[b] < costAct+map[kh][kw])){
							parent[b] = nextNode;
							openListG[b] = costAct + map[kh][kw];
							openListF[b] = openListG[b] + openListH[b];
						}
						//if node is not in the open list and not in the closed list
						if (!open[b] && !closed[b]){
							open[b] = true;
							openListG[b] = costAct + map[ah][aw];
							openListF[b] = openListG[b] + openListH[b];
							parent[b] = nextNode;
							openList[openListCounter] = b;
							openListCounter++;
						}
					}
				}
			}
		}while (!arrived); //while no goal node has been found
		
		//constructing the way using parents:
		way[0] = goalNode;
		b = 1;
		int nodeAct = goalNode;
		int wayLength = 0;
		while (nodeAct != startNode && parent[nodeAct] != -2) {
			way[b] = parent[nodeAct];
			nodeAct = parent[nodeAct];
			wayLength = b;
			b++;
		}
		
		//turn the way around: ;)
		// should return an arraylist of waypoints (x,y) coordinates in the array
		
		int[] retWay = new int[wayLength+1];
		for(int i = 0; i <= wayLength; i++){ 
			retWay[i] = way[wayLength - i];
			//i.e. add the (12,20) as a waypoint
			route.add(new Point(12,20));
		}
		return route;
	}
	
	//Calculates the heuristics
	public static int calculateh(int g, int s, int w, int h) {
		int gh = g/w;
		int gw = g - gh*w;
		int sh = s/w;
		int sw = s - sh*w;

		return Math.max(Math.abs(sh-gh), Math.abs(sw-gw));
	}
}
