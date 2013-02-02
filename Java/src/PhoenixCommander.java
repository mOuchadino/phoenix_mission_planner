import java.util.ArrayList;
import java.awt.*;
import java.awt.event.*;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Graphics;
import java.awt.Point;

import javax.swing.*;


public class PhoenixCommander extends JFrame {
  
  private static final long serialVersionUID = 1L;
  private MapPanel mapPanel;
  private JPanel controlPanel;
  private ArrayList<Point> wayPoints = new ArrayList<Point>();
  private int anz = 40;                                 // Anzahl der Punkte
  private int smoothing = 1;                            // SFaktor (1 bis 10)
  double wdata = 0.5, wsmooth = 0.1, change = 0.01;     // mehr Smoothing
  private int path [][] = new int [anz][2];             // Pfad vorher
  private int pathf [][] = new int [anz][2];            // Pfad nachher
  private JCheckBox punkte [] = new JCheckBox[anz];
  private JCheckBox punktef [] = new JCheckBox[anz];  
  private double winkel_array [] = new double [anz];    // Winkel 
  private int wrong [] = new int[anz/4];                            
  private Button bu1 = new Button("single");            // einzeln nachbessern
  private Button bu2 = new Button("all");               // smoothing ++ ->alles
  private Button bu3 = new Button("show points");       // Punkte anzeigen
  
  
  public PhoenixCommander(String title) { 
    super(title);
    setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    setSize(900, 600);
    Dimension d = Toolkit.getDefaultToolkit().getScreenSize();
    setLocation((d.width-getSize().width)/2, (d.height-getSize().height)/2);
    Container cp = getContentPane();
    cp.setLayout(new BorderLayout(5,5));
    mapPanel = new MapPanel("img/floorplan.png");
    mapPanel.addMouseListener(new MouseAdapter() {
	    public void mouseClicked(MouseEvent e) {
	    	wayPoints.add(e.getPoint());
		    renderWaypoints();
	    }
	});    
    cp.add(mapPanel, BorderLayout.CENTER);
    controlPanel= new JPanel();
    controlPanel.setSize(100, 300);
    controlPanel.setLayout(new BoxLayout(controlPanel, BoxLayout.X_AXIS));
    cp.add(controlPanel, BorderLayout.PAGE_END);

    setVisible(true);

    
    path [0][0] = 10;       path [0][1] = 20;
    path [1][0] = 40;       path [1][1] = 20;
    path [2][0] = 70;       path [2][1] = 30;
    path [3][0] = 80;       path [3][1] = 50;
    path [4][0] = 80;       path [4][1] = 80;
    path [5][0] = 60;       path [5][1] = 100;
    path [6][0] = 80;       path [6][1] = 120;
    path [7][0] = 100;      path [7][1] = 150;
    path [8][0] = 120;      path [8][1] = 150;
    path [9][0] = 150;      path [9][1] = 120;
    
    path [10][0] = 160;     path [10][1] = 160;
    path [11][0] = 200;     path [11][1] = 200;
    path [12][0] = 200;     path [12][1] = 230;
    path [13][0] = 200;     path [13][1] = 250;
    path [14][0] = 150;     path [14][1] = 240;
    path [15][0] = 140;     path [15][1] = 180;
    path [16][0] = 160;     path [16][1] = 100;
    path [17][0] = 200;     path [17][1] = 110;
    path [18][0] = 230;     path [18][1] = 150;
    path [19][0] = 300;     path [19][1] = 200;
    
    path [20][0] = 300;     path [20][1] = 200;
    path [21][0] = 350;     path [21][1] = 200;
    path [22][0] = 400;     path [22][1] = 200;
    path [23][0] = 400;     path [23][1] = 250;
    path [24][0] = 400;     path [24][1] = 300;
    path [25][0] = 350;     path [25][1] = 300;
    path [26][0] = 300;     path [26][1] = 300;
    path [27][0] = 300;     path [27][1] = 250;
    path [28][0] = 280;     path [28][1] = 200;
    path [29][0] = 500;     path [29][1] = 100;
    
    path [30][0] = 600;     path [30][1] = 100;
    path [31][0] = 700;     path [31][1] = 100;
    path [32][0] = 700;     path [32][1] = 200;
    path [33][0] = 700;     path [33][1] = 300;
    path [34][0] = 600;     path [34][1] = 300;
    path [35][0] = 600;     path [35][1] = 400;
    path [36][0] = 400;     path [36][1] = 400; 
    path [37][0] = 400;     path [37][1] = 460;
    path [38][0] = 300;     path [38][1] = 420;
    path [39][0] = 100;     path [39][1] = 300;
    
    for (int g=0; g<anz ; g++ ) {
      for (int h=0; h<2 ; h++) {
        pathf[g][h] = path[g][h];
      }
    } 
    
    winkel();
    ausgeben();
    
    int summe = 0;
    for (int u=0; u<(anz-1) ; u++) {
    summe += path[u+1][0]-path[u][1] ;}
    System.out.println("Summe: " + summe);
    
    for (int i=0; i<anz ; i++ ) {
      punkte[i] = new JCheckBox();
      punktef[i] = new JCheckBox();
    } 
    
    do_traject();
    winkel();
    ausgeben();
    //find_wrong();
    //winkel();
    //ausgeben();
    
    summe = 0;
    for (int u=0; u<(anz-1) ; u++) {
      summe += pathf[u+1][0] - pathf[u][1] ;
    } System.out.println("Summe: " + summe);
    
    /*for (int n=0; n<anz ; n++ ) {  //würd ich rauslassen
      punkte[n].setBounds(path[n][0], path[n][1], 20, 20);
      punkte[n].setSelected(true);    
      punkte[n].setVisible(true);           
      cp.add(punkte[n]);                      //Kommentieren m�glich
      
      punktef[n].setBounds(pathf[n][0], pathf[n][1], 20, 20);               
      punkte[n].setVisible(true);
      cp.add(punktef[n]);                    //Ent-Kommentieren f�r Checkboxen
      repaint();
    }*/
    
    bu1.addActionListener(new ActionListener() { 
      public void actionPerformed(ActionEvent evt) { 
      bu1_ActionPerformed(evt);}});
    controlPanel.add(bu1);
    
    bu2.addActionListener(new ActionListener() { 
      public void actionPerformed(ActionEvent evt) { 
      bu2_ActionPerformed(evt);}});
    controlPanel.add(bu2);
    
    bu3.addActionListener(new ActionListener() { 
      public void actionPerformed(ActionEvent evt) { 
      bu3_ActionPerformed(evt);}});
    controlPanel.add(bu3);
    repaint();

    setVisible(true);
  } 
  
  public void do_traject() {
    int newpath [][];
    newpath = pathf;
    for (int z=0; z<smoothing ; z++) { 
      for (int a=1; a<(anz-1) ; a++ ) {
        for (int b=0; b<2 ; b++ ) {
          int aux = newpath [a][b];
          newpath[a][b] += wdata*(pathf[a][b] - newpath[a][b]);
          newpath[a][b] += wsmooth*(newpath[a-1][b]+newpath[a+1][b]-(2.0*newpath[a][b]));
          change += Math.abs(aux-newpath[a][b]);
        }     
      } 
    }
    pathf = newpath;
  }
  
  public void winkel() {
    
    for (int a=0; a< (anz-2) ; a++) {
      System.out.print("Winkel delta: ");
      int ax = pathf[a+1][0] - pathf[a][0];
      int ay = pathf[a+1][1] - pathf[a][1];
      int bx = pathf[a+2][0] - pathf[a+1][0];
      int by = pathf[a+2][1] - pathf[a+1][1];
      
      double alpha = (180/3.1415)*(Math.cos((ax*bx + ay*by)/ (Math.sqrt(ax*ax+ay*ay) * Math.sqrt(bx*bx+by*by) )));
      winkel_array[a] = alpha;
      System.out.println(" " + alpha);
    } 
  }
  
  public void find_wrong() {
    for (int i=0; i< (anz/4) ; i++ ) { wrong[i] = 0;} 
    for (int x=0; x<anz ; x++) {
      if (winkel_array[x]>50) {
        System.out.println("Fehler an Punkt:  " + x + " Wert: "+ winkel_array[x]);
        do_single_trajectory(x+1);
      }
    } 
  } 
  
  public void do_single_trajectory(int a) {
    for (int x=0; x<2; x++ ) {
      //pathf[a][x] += wdata*(pathf[a][x] - newpath[a][b]);
      //pathf[a][x] += wsmooth*(newpath[a-1][b]+newpath[a+1][b]-(2.0*newpath[a][b]));
      
      //pathf[a][x] += wdata*(pathf[a][x] - path[a][x]);
      //pathf[a][x] += wsmooth*(path[a-1][x]+path[a+1][x]-(2.0*path[a][x]));
      
      pathf[a][x] += wdata*(pathf[a][x] - pathf[a][x]);
      pathf[a][x] += wsmooth*(pathf[a-1][x]+pathf[a+1][x]-(2.0*pathf[a][x]));
      
    } 
  }  
  
  public void paint(Graphics g) {
    super.paint(g);
    int a = pathf[0][0];
    int b = pathf[0][1]+10;
    
    Graphics2D g2d = (Graphics2D)g;
    //g.setColor(new Color(0, 0, 255));
    g.setColor(new Color(0, 0, 0));
    for (int f=0; f<(anz-1); f++) {
      g.drawLine(path[f][0]+a, path[f][1]+b, path[f+1][0]+a, path[f+1][1]+b);
    }
    
    g.setColor(new Color(255, 0, 0));
    BasicStroke stroke3= new BasicStroke(6.0f, BasicStroke.CAP_SQUARE, BasicStroke.JOIN_BEVEL);
    g2d.setStroke(stroke3);
    
    for (int f=0; f<(anz-1); f++) {
      if (winkel_array[f] < 35) { g.setColor(new Color(0, 255, 0));}
      else if(winkel_array[f] >= 35 && winkel_array[f] <50) { g.setColor(new Color(0, 0, 255));}
      else if(winkel_array[f] > 50) { g.setColor(new Color(255, 0, 0));}
      
      //g.setColor(new Color(255, 0, 0));
      g.drawLine(pathf[f][0]+a, pathf[f][1]+b, pathf[f+1][0]+a, pathf[f+1][1]+b);
    }
    
    Point previousPoint=null;
    g.setColor(Color.blue);
    for (Point p : wayPoints){
    	if (previousPoint!=null)
    	{
    		g.drawLine(previousPoint.x, previousPoint.y+16, p.x, p.y+16);
    	}
    	previousPoint=p;
    }
    
  }
  
  public void ausgeben() {
    for (int e=0; e<anz; e++) {
      System.out.println("Punkt _ _ _:  "+e+"  " + path[e][0]+ " -- "+ path[e][1]);
      System.out.println("Punktfinal_:  "+e+"  " + pathf[e][0]+ " -- "+ pathf[e][1]);      
    }  System.out.println("\n");  
  }
  
  public void renderWaypoints(){
	    ImageIcon waypointIcon= new ImageIcon("img/waypoint.png");
	    for (Point p : wayPoints){
	    	JLabel label=new JLabel(waypointIcon);
	    	label.setSize(32,32);
	    	label.setLocation(p.x-16,p.y-32);
	        mapPanel.add(label);
	    }
	    repaint();
	    mapPanel.repaint();
}
  
  public static void main(String[] args) {
    new PhoenixCommander("Phoenix Commander");
  } 
  
  public void bu1_ActionPerformed(ActionEvent evt) {
    find_wrong();
    winkel();
    ausgeben();
    repaint();
  } 
  public void bu2_ActionPerformed(ActionEvent evt) {
    //smoothing += 1;
    do_traject();
    winkel();
    ausgeben();
    repaint();
  } 
  public void bu3_ActionPerformed(ActionEvent evt) {
    for (int n=0; n<anz ; n++ ) {      
      
      if (punkte[0].isVisible()) {
        punkte[n].setVisible(false);                          
        punktef[n].setVisible(false);
      }
      else {
        punktef[n].setBounds(pathf[n][0], pathf[n][1], 20, 20);  
        punkte[n].setVisible(true);                          
        punktef[n].setVisible(true);
      } 
      repaint();
    }
    
    repaint();
  } 
  
  
} 