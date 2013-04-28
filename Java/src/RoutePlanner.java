import java.awt.Point;
import java.util.ArrayList;

public class RoutePlanner {
	
	private int[][] map;
	private ArrayList<Point> route=new ArrayList<Point>();
	
	public RoutePlanner(int[][] map) {
		this.map=map;
	}
	
	//The path finding method
	//Takes as arguments the first node s, the goal node g, the map, coordinates of corners left-up and right-down 
	//Returns the shortest way in form of a double-array of the form (x-coordinate/y-coordinate)
	
	public ArrayList<Point> planRoute(Point start, Point goal) {
	    double g_x = goal.x;  //goal node
	    double g_y = goal.y;
	    double s_x = start.x;  //start node
	    double s_y = start.y;
	    //coordinates of left up corner
	    double lo_x = 0;
	    double lo_y = 0;
	    //coordinates of right down corner, are those correct here pls doublecheck???? 
	    double rd_x = map[0].length;
	    double rd_y = -map.length;
		//Initialization:
		int w = map[0].length; //width of the map (in bit)
		int h = map.length;  //height of the map (in bit)
		int g = arrayValue(lo_x, rd_x, g_x, w, lo_y, rd_y, g_y, h);  //calculates the number in the array of the goal
		int s = arrayValue(lo_x, rd_x, s_x, w, lo_y, rd_y, s_y, h);  //calculates the number in the array of the start
		
		boolean arrived = false;
		int openListCounter = 1;
		int nextNode;
		int b = 0; 
		int savei = 0;
		boolean check = false; //for choosing node from openList
		
		int costAct = 0;  //actual costs
		int[] openList = new int[h*w];
		int[]way = new int[h*w];
		Node[] nodeList = new Node[h*w];


		//Initializations of different arrays
		for(int i = 0; i < h*w; i++) {
			way[i] = -1;
			openList[i] = -1;
			nodeList[i] = new Node();
			nodeList[i].y_array = i/w;
			nodeList[i].x_array = i - nodeList[i].y_array*w;
			nodeList[i].y_coordinate = calcX(lo_x, rd_x, w, nodeList[i]);
			nodeList[i].x_coordinate = calcY(lo_y, rd_y, h, nodeList[i]);
		}
		for(int i = 0; i < h*w; i++) {
			nodeList[i].hvalue = calculateh(g, i, w, h, nodeList);
		}
		
		System.out.println("g: "+g+ ", s: "+ s);
		
		//insert the start node into the open list:
		nodeList[s].open = true;
		openList[0] = s;
		nodeList[s].gvalue = 0;
		nodeList[s].fvalue = nodeList[s].hvalue;
		nextNode = s;


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
					if (nodeList[openList[i]].fvalue < nodeList[nextNode].fvalue){ //ggf Minimum verwenden
						nextNode = openList[i];
						savei = i;
					}
				}
			}
			check = true;
			if (nextNode == g){
				costAct = nodeList[g].gvalue;
				arrived = true;
			}
			else {
				nodeList[nextNode].closed = true;
				nodeList[nextNode].open = false;
				openList[savei] = -1;
				costAct = nodeList[nextNode].gvalue;
				//Consider the neighbors of the actual node:
				int kh = nextNode/w;
				int kw = nextNode - Math.max(0, kh)*w;

				
				for(int ah = Math.max(0, kh-1); ah <= Math.min(h-1, kh+1); ah++) {
					for(int aw = Math.max(0, kw-1); aw <= Math.min(w-1, kw+1); aw++) {
						
						b = w*Math.max(0, ah) + aw;
						
						//if node is in the open list and costAct is lower than it's g
						if (nodeList[b].open && (nodeList[b].gvalue < costAct+map[kh][kw])){
							nodeList[b].parent = nextNode;
							nodeList[b].gvalue = costAct + map[kh][kw];
							nodeList[b].fvalue = nodeList[b].gvalue + nodeList[b].hvalue;
						}
						//if node is not in the open list and not in the closed list
						if (!nodeList[b].open && !nodeList[b].closed){
							nodeList[b].open = true;
							nodeList[b].gvalue = costAct + map[ah][aw];
							nodeList[b].fvalue = nodeList[b].gvalue + nodeList[b].hvalue;
							nodeList[b].parent = nextNode;
							openList[openListCounter] = b;
							openListCounter++;
						}
					}
				}
			}
		}while (!arrived); //while no goal node has been found
		
		//constructing the way using parents:
		way[0] = g;
		b = 1;
		int nodeAct = g;
		int wayLength = 0;
		while (nodeAct != s && nodeList[nodeAct].parent != -2) {
			way[b] = nodeList[nodeAct].parent;
			nodeAct = nodeList[nodeAct].parent;
			wayLength = b;
			b++;
		}
		
		//turn the way around and use coordinates:
		//somehow we lose the x coordinate after this

		double[][] retWay = new double[wayLength+1][2];
		for(int i = 0; i <= wayLength; i++){ 
			retWay[i][0] = nodeList[way[wayLength - i]].x_coordinate;
			retWay[i][1] = nodeList[way[wayLength - i]].y_coordinate;
		}
		
		
		for(int i = 0; i <= wayLength; i++){ 
			route.add(new Point((int)retWay[i][0],(int)retWay[i][1]));
		}
	
		return route;
	}



	//Calculates the heuristics
	public static int calculateh(int g, int s, int w, int h, Node[] nodeList) {
		return Math.max(Math.abs(nodeList[s].y_array-nodeList[g].y_array), Math.abs(nodeList[s].x_array-nodeList[g].x_array));
	}
	
	//Calculates the x-coordinate
	//Takes the x-coordinate of the two corners, the width of the Node-array and the considered node as input
	public static double calcX(double lo_x, double rd_x, int w, Node node1) {
		//calculate the width of the area:
		double w1 = rd_x - lo_x;
		double help1 = (double)-node1.x_array/(w-1); 

		return (double)Math.round((help1*w1 + lo_x)*100)/100;	
	}
	
	//Calculates the y-coordinate
	//Takes the y-coordinate of the two corners, the height of the Node-array and the considered node as input
	public static double calcY(double lo_y, double rd_y, int h, Node node1) {
		//calculate the height of the area:
		double h1 = lo_y - rd_y;
		double help1 = (double)node1.y_array/(h-1); 

		return (double)Math.round((help1*h1 + lo_y)*100)/100;	
	}
	
	//Calculates the number in the array of the node:
	public static int arrayValue(double lo_x, double rd_x, double g_x, int w, double lo_y, double rd_y, double g_y, int h) {
		//Calculates the x-value in the array; possibly not exact at the end, but we can't change this now
		//calculate the width of the area:
		double w1 = rd_x - lo_x;
		double help1 = (g_x-lo_x)/w1; 

		int aw = (int)(help1*(w-1));	
		System.out.println(aw);
	    //Calculates the y-value in the array
		//calculate the width of the area:
		double h1 = lo_y - rd_y;
		double help2 = (lo_y-g_y)/h1; 

		int ah = (int)(help2*(h-1));	
		System.out.println(ah);
		return w*Math.max(0, ah) + aw;
	}

	public ArrayList<Point> getRoute() {
		return route;
	}
}

//implementation the nodes as objects of type Node
class Node {

	boolean closed;
	boolean open;
	int fvalue;
	int gvalue;
	int hvalue;
	int parent;
	int x_array;  //position in the array    
  int y_array;    
	double x_coordinate;   //coordinates in the used coordinate system
  double y_coordinate; 

  //Standard constructor:
  Node()
  {
  	this.closed = false;
  	this.open = false;
  	this.fvalue = 0;
  	this.gvalue = 0;
  	this.hvalue = 0; //has to be set in the program
  	this.parent = -2;
      this.x_coordinate = 0; //has to be set in the program
      this.y_coordinate = 0; //has to be set in the program
  }
}

