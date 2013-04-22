import java.awt.Point;
import java.util.ArrayList;


public class TrajectoryPlanner {
	  private ArrayList<Point> wayPoints;
	  private ArrayList<Point> wayPointsSmoothed;
	  private ArrayList<Integer> winkel = new ArrayList<Integer>();
	  double wdata = 0.5, wsmooth = 0.1, change = 0.01;     // smoothing parameter, what exactly do they do thomas?

	
	public ArrayList<Point> smootheTrajectory(ArrayList<Point> wps) {
		wayPoints= new ArrayList<Point>(wps);
	    ArrayList<Point> wayPointsTmp = new ArrayList<Point>(wps);
	    for(int i=1; i<wayPointsTmp.size()-1; i++)
	    {
	      wayPointsTmp.get(i).x += wdata*(wayPoints.get(i).x-wayPointsTmp.get(i).x);
	      wayPointsTmp.get(i).x += wsmooth*(wayPointsTmp.get(i-1).x + 
	      wayPointsTmp.get(i+1).x - (2.0*wayPointsTmp.get(i).x));
	      
	      wayPointsTmp.get(i).y += wdata*(wayPoints.get(i).y-wayPointsTmp.get(i).y);
	      wayPointsTmp.get(i).y += wsmooth*(wayPointsTmp.get(i-1).y + 
	      wayPointsTmp.get(i+1).y - (2.0*wayPointsTmp.get(i).y));
	    }
	    wayPointsSmoothed=new ArrayList<Point>(wayPointsTmp);
	    return wayPointsTmp;
	  }
	  
	  public void winkelMessen() {
	    winkel.clear();
	    for (int a=0; a<wayPointsSmoothed.size()-1; a++) { //sollte das ned bis wayPointsSmoothed.size gehen?
	      System.out.print("Winkel delta: ");
	      int ax = wayPointsSmoothed.get(a+1).x - wayPointsSmoothed.get(a).x;
	      int ay = wayPointsSmoothed.get(a+1).y - wayPointsSmoothed.get(a).y;
	      int bx = wayPointsSmoothed.get(a+2).x - wayPointsSmoothed.get(a+1).x;
	      int by = wayPointsSmoothed.get(a+2).y - wayPointsSmoothed.get(a+1).y;
	      //double alpha = (180/3.1415)*(Math.acos((ax*bx+ay*by)/((double)(Math.sqrt(ax*ax+ay*ay)*Math.sqrt(bx*bx+by*by)))));
	      double alpha = (180/Math.PI) * Math.acos((ax*bx+ay*by*1.0)/(double)(Math.sqrt(ax*ax+ay*ay*1.0)*Math.sqrt(bx*bx+by*by*1.0)))    ;
	      winkel.add(new Integer((int) alpha));
	      System.out.println(" " + alpha);
	    } 
	  }
	  
	  public void printresult(String what)
	  { 
	    System.out.println("printresult "+ what);
	    for(int i=0; i<wayPoints.size() ;i++ )   //for(Point p : wayPoints)
	    {
	      System.out.print("wayPoint: "+i+"   x: "+wayPoints.get(i).x+"   y: "+wayPoints.get(i).y +"     ");
	      System.out.println("wayPointS: "+i+"   x: "+wayPointsSmoothed.get(i).x+"   y: "+wayPointsSmoothed.get(i).y);
	    }
	  }
}
