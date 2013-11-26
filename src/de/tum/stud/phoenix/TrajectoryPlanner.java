package de.tum.stud.phoenix;
import geometry_msgs.Point;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

//local the trajectory planner should eventually account for the physical constraints of the phoenix
//and generates feasible local trajectories in real-time

public class TrajectoryPlanner {
	private ArrayList<Point> wayPoints;
	private ArrayList<Float> angles = new ArrayList<Float>();
	private ArrayList<Vector3d> velocityVectors = new ArrayList<Vector3d>();

	public Trajectory plan(Path path){
		Trajectory trajectory = new Trajectory();
		for(ArrayList<Point> segment:path.getSegments()){
			for(Point point:segment){
				TimeStep timeStep=new TimeStep();
				timeStep.pose.setPosition(point);				
				trajectory.segments.add(timeStep);
			}
		}
		//now we can calculate the yaws/psi as
		double ax,ay,bx,by,psi;
		for (int i = 0; i < trajectory.segments.size(); i++) {
			//von 0 nach 1
			ax=trajectory.segments.get(i).pose.getPosition().getX();
			ay=trajectory.segments.get(i).pose.getPosition().getY();
			try {

				bx=trajectory.segments.get(i+1).pose.getPosition().getX();
				by=trajectory.segments.get(i+1).pose.getPosition().getY();
			}
			catch(IndexOutOfBoundsException e){
				bx=trajectory.segments.get(i).pose.getPosition().getX();
				by=trajectory.segments.get(i).pose.getPosition().getY();
			}

			psi = Math.toDegrees(Math.atan2(by,bx) - Math.atan2(ay,ax));
			//trajectory.segments.get(i+1).pose.setOrientation(arg0);
			System.out.println("psi "+psi);
		}
		
		return trajectory;
	}

	public ArrayList<Float> calculatePsiDot(ArrayList<Float> psis) {
		ArrayList<Float> psidots=new ArrayList<Float>();
		for (int i = 0; i < psis.size()-1; i++) {
			psidots.add(psis.get(i+1)-psis.get(i));
		}
		return psidots;
	}

	public ArrayList<Vector3d> calculateVelocities(ArrayList<Point> wps) {
		Vector3d v=new Vector3d();
		velocityVectors.clear();

		for (int a=0; a<wps.size()-1; a++) {
			v.x=wps.get(a+1).getX()-wps.get(a).getX();
			v.y=wps.get(a+1).getY()-wps.get(a).getY();
			v.z=0; //bz-az;
			velocityVectors.add(v);
		}

		return velocityVectors;
	}
}