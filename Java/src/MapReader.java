import java.awt.AlphaComposite;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.image.*;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.imageio.ImageIO;

public class MapReader {

	BufferedImage mapImage;
	BufferedImage resizedMapImage;
	private double scaleFactor=1.0;




	int[][] map;

	public MapReader(String mapName) {

		try{
			mapImage = ImageIO.read(new File(mapName));
			resizedMapImage = resizeImageWithHint(mapImage, mapImage.getType(), (int)(mapImage.getWidth()*scaleFactor), (int)(mapImage.getHeight()*scaleFactor));

		}catch(IOException e){
			System.out.println(e.getMessage());
		}

		int h = resizedMapImage.getHeight();
		int w = resizedMapImage.getWidth();
		System.out.println(h + " " + w);

		map = new int[w][h];
		List<Integer> basket = new ArrayList<Integer>();  

		for(int i = 0; i < w; i++) {
			for(int j = 0; j < h; j++) {
				int pixel = resizedMapImage.getRGB(i, j);
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

	private BufferedImage resizeImageWithHint(BufferedImage originalImage, int type, int width, int height){
		BufferedImage resizedImage = new BufferedImage(width, height, type);
		Graphics2D g = resizedImage.createGraphics();
		g.drawImage(originalImage, 0, 0, width, height, null);
		g.dispose();	
		g.setComposite(AlphaComposite.Src);
		g.setRenderingHint(RenderingHints.KEY_INTERPOLATION,RenderingHints.VALUE_INTERPOLATION_BILINEAR);
		g.setRenderingHint(RenderingHints.KEY_RENDERING,RenderingHints.VALUE_RENDER_QUALITY);
		g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON);

		return resizedImage;
	}
	
	public BufferedImage getResizedMapImage() {
		return resizedMapImage;
	}

	public void setResizedMapImage(BufferedImage resizedMapImage) {
		this.resizedMapImage = resizedMapImage;
	}
}
