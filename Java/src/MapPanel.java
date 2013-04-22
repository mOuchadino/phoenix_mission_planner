import java.awt.Graphics;
import java.awt.image.BufferedImage;
import javax.swing.JPanel;


public class MapPanel extends JPanel{

	private static final long serialVersionUID = -6348356140496920472L;
	private BufferedImage resizedImage;

    public MapPanel(BufferedImage img) {               
    	   resizedImage=img;
    }
    
    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        g.drawImage(resizedImage, 0, 0, null);
    }
}
