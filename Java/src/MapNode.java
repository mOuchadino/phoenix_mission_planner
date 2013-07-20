import java.awt.Point;

public class MapNode implements Comparable<MapNode>
	{
		Point location;
		int cost;
		int nextObstacleDistance;
		int g;
		double h;
		double f;
		boolean goal;
		boolean visited;
		boolean obstacle;
		boolean isGVD;

		MapNode parent;
		MapNode waveOrigin;


		@Override
		public int compareTo(MapNode compareObject)
		{
			if (f < compareObject.f)
				return -1;
			else if (f == compareObject.f)
				return 0;
			else
				return 1;
		}
	}