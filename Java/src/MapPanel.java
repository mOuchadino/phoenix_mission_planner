import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.JPanel;


public class MapPanel extends JPanel{

	private static final long serialVersionUID = -6348356140496920472L;
	private BufferedImage image;

    public MapPanel(String filename) {
       try {                
          image = ImageIO.read(new File(filename));
          setPreferredSize(new Dimension(image.getWidth(),image.getHeight()));
       } catch (IOException ex) {
            // handle exception...
       }
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        g.drawImage(image, 0, 0, null); // see javadoc for more info on the parameters            
    }

}
