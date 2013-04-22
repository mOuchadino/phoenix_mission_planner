import java.awt.image.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class MapReader {

	int[][] map;
	
	  public MapReader(BufferedImage img) {
			    int h = img.getHeight();
			    int w = img.getWidth();
			    System.out.println(h + " " + w);
			    
			    map = new int[w][h];
			    List<Integer> basket = new ArrayList<Integer>();  
			    
			    for(int i = 0; i < w; i++) {
			    	for(int j = 0; j < h; j++) {
			    		int pixel = img.getRGB(i, j);
			    		map[i][j] = (pixel<-16)?9:1;
			    		if(!basket.contains(map[i][j])){
			    			basket.add(map[i][j]);
			    		}
			    	}
			    }
			    Collections.sort(basket);
			    for (Integer i : basket) {
					System.out.println(Integer.toHexString(i) + ":" + i);
				}
	}

	public int[][] getMap() {
		return map;
	}
}
