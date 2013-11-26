package de.tum.stud.phoenix;
import java.awt.AlphaComposite;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.image.*;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;

//TODO: Convolve with gaussian Kernel to smoothe path

public class MapReader {

	BufferedImage mapImage,resizedMapImage;
	private double scaleFactor=1.0; //20 for tinyfloorplan
	private double greyThreshold=0.6; //for grey values smaller than the threshold we have an obstacle
	private int maxDia=15; //max smoothing diameter
	private int[][] map;

	public MapReader(String mapName) {
		try
		{
			mapImage = ImageIO.read(new File(mapName));
			resizedMapImage = resizeImageWithHint(mapImage, mapImage.getType(), (int)(mapImage.getWidth()*scaleFactor), (int)(mapImage.getHeight()*scaleFactor));
		}
		catch(IOException e){
			System.out.println(e.getMessage());
		}

		int h = resizedMapImage.getHeight();
		int w = resizedMapImage.getWidth();
		System.out.println("ResizedMap "+w + "x" + h);

		map = new int[w][h];

		for(int i = 0; i < w; i++) {
			for(int j = 0; j < h; j++) {
				Color color = new Color(resizedMapImage.getRGB(i, j));
				float[] greyPixel = Color.RGBtoHSB(color.getRed(),color.getGreen(),color.getBlue(),null);
				if(greyPixel[2]<greyThreshold){
					map[i][j]=2000;
					//treat surrounding with safety

					for (int dia=1;dia<maxDia;dia++){
						for(int r=-dia;r<=dia;r++){
							for(int c=-dia;c<=dia;c++){
								if(r==0 && c==0)//this is where we started;
								{
									continue; 
								}
								try {
									map[i+r][j+c]+=(maxDia-dia);
									if (map[i+r][j+c]>2000)
										map[i+r][j+c]=2000;
								}
								catch(ArrayIndexOutOfBoundsException e){
								}
							}
						}
					}	
				} else {
					map[i][j]+=1;
				}
			}
		}
	}

	public int[][] getCostMap() {
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