import java.awt.Point;
import java.util.ArrayList;

//local the trajectory planner accounts for the constraints and generates feasible local trajectories in real-time (collision avoidance)

public class TrajectoryPlanner {
	  private ArrayList<Point> wayPoints=new ArrayList<Point>();
	  private ArrayList<Point> wayPointsSmoothed;
	  private ArrayList<Float> angles = new ArrayList<Float>();
	  double wdata = 0.5, wsmooth = 0.1, change = 0.01;     // smoothing parameter, what exactly do they do thomas?

	
	public ArrayList<Point> smootheTrajectory(Route route) {
	    ArrayList<Point> wayPointsTmp = new ArrayList<Point>();
		for(ArrayList<Point> segment:route.getSegments()){
			wayPoints.addAll(segment);
			wayPointsTmp.addAll(segment);
			}
		
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
	  
	  public ArrayList<Float> calculateAngles(ArrayList<Point> wps) {
		 wayPointsSmoothed=wps;
		System.out.println("Total of "+wayPointsSmoothed.size()+" interpolated waypoints found, calculating angles");
	    angles.clear();
	    float alpha=0;
	    int ax,ay,bx,by;
	    //for the first angle use derivation from north
	    bx = wayPointsSmoothed.get(1).x - wayPointsSmoothed.get(0).x;
	    by = wayPointsSmoothed.get(1).y - wayPointsSmoothed.get(0).y;
	    alpha =(float) Math.toDegrees(Math.atan2(by,bx) - Math.atan2(-1,0));
	    angles.add( alpha);
	    System.out.println("at wp 0 delta yaw "+ alpha); //drehwinkel in der horizontalen ebene bei flugzeugen yaw

	    
	    for (int a=0; a<wayPointsSmoothed.size()-2; a++) {
	      ax = wayPointsSmoothed.get(a+1).x - wayPointsSmoothed.get(a).x;
	      ay = wayPointsSmoothed.get(a+1).y - wayPointsSmoothed.get(a).y;
	      bx = wayPointsSmoothed.get(a+2).x - wayPointsSmoothed.get(a+1).x;
	      by = wayPointsSmoothed.get(a+2).y - wayPointsSmoothed.get(a+1).y;
	      alpha =(float) Math.toDegrees(Math.atan2(by,bx) - Math.atan2(ay,ax));
	      angles.add( alpha);
	      System.out.println("at wp " +a+1+ " delta yaw "+ alpha); //drehwinkel in der horizontalen ebene bei flugzeugen yaw
	    }
	    //FIXME
	    angles.add( (float) 0.0);
	    angles.add( (float) 0.0);

	    return angles;
	  }
}