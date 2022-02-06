/**
 **********************************************************************************************************************
 * @file       canhap501_lab2_Jeremy.pde
 * @author     Jérémy
 * @version    V1.0.0
 * @date       4-February-2022
 * @brief      
 **********************************************************************************************************************
 * @attention!!!
 * The base of this code was taken from:
 * @file       Maze.pde
 * @author     Elie Hymowitz, Steve Ding, Colin Gallacher
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
 *
 **********************************************************************************************************************
 */

/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  

/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 

/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/

/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 

/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 32.5;  
float             worldHeight                         = 20.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2

/* Initialization of virtual tool */
HVirtualCoupling  s;

// Starting triangle
FPoly tri_start;

// Game Object
FBox box1;
FBox box2;
FBox box3;
FBox box4;
boolean b1Showing = false;
boolean b2Showing = false;
boolean b3Showing = false;
boolean b4Showing = false;
int box1Timer;
int box2Timer;
int box3Timer;
int box4Timer;

FBox vicsousArea1;
boolean v1Showing = false;
FBox vicsousArea2;
boolean v2Showing = false;

FCircle circle1;
boolean circle1Hit;

/* define start and stop button */
// FCircle           c1;
// FCircle           c2;

/* define game ball */
// FCircle           g2;
// FBox              g1;

boolean gameStart = false;
boolean gameOver = false;
int counter = 0;
int timeOutDelay = 5000;
int targetsTotal = -1;
int targetsHit = 0;
int gameOverDifference = 10;

int timer = 0;
int rotation = 1;

/* text font */
PFont f;

/* end elements definition *********************************************************************************************/  

/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  size(1300, 800);
  f = createFont("Arial", 16, true);

  /* device setup */
  haplyBoard          = new Board(this, "Com7", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  widgetOne.device_set_parameters();
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
    
    
  // Starting point / trigger
  tri_start = new FPoly();
  tri_start.vertex(edgeTopLeftX + 0.75, edgeTopLeftY + 0.75);
  tri_start.vertex(edgeTopLeftX + 4  * 0.75, edgeTopLeftY + 0.75);
  tri_start.vertex(edgeTopLeftX + 0.75, edgeTopLeftY + 4 * 0.75);
  tri_start.setStaticBody(true);
  // tri_start.setPosition(edgeTopLeftX+1, edgeTopLeftY+1);
  world.add(tri_start);

  circle1 = new FCircle(1.0);
  box1 = new FBox(1.0, 3.0);
  box1.setPosition(10, 10);
  box1.setStaticBody(true);
  box1.setFill(0);
  box1.setNoStroke();
  box1Timer = 5;

  box2 = new FBox(4.0, 1.0);
  box2.setPosition(27, 12);
  box2.setStaticBody(true);
  box2.setFill(0);
  box2.setNoStroke();
  box2Timer = int(random(6,12));

  box3 = new FBox(4.0, 1.0);
  box3.setPosition(15, 15);
  box3.setStaticBody(true);
  box3.setFill(0);
  box3.setNoStroke();
  box3Timer = int(random(15, 21));

  box4 = new FBox(4.0, 1.0);
  box4.setPosition(20, 5);
  box4.setStaticBody(true);
  box4.setFill(0);
  box4.setNoStroke();
  box4Timer = int(random(24,30));
   
  vicsousArea1 = new FBox(4, worldHeight-4);
  vicsousArea1.setPosition(worldWidth/2, worldHeight/2 + 1);
  vicsousArea1.setFill(200, 200, 200, 80);
  vicsousArea1.setSensor(true);
  vicsousArea1.setNoStroke();
  vicsousArea1.setStatic(true);

  vicsousArea2 = new FBox(worldWidth-4, 4);
  vicsousArea2.setPosition(worldWidth/2, worldHeight/2);
  vicsousArea2.setFill(200, 200, 200, 80);
  vicsousArea2.setSensor(true);
  vicsousArea2.setNoStroke();
  vicsousArea2.setStatic(true);
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(255,0,0); 
  s.h_avatar.setSensor(true);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  // world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  world.draw();
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/

/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    textFont(f, 22);
 
    if(gameStart){
      fill(0, 0, 0);
      textAlign(CENTER);
      text("Touch the circles, avoid the boxes.", width/2, 60);
      textAlign(CENTER);
      text("You've hit " + targetsHit + " on " + targetsTotal + " targets...", width/2, 90);
    }
    else{
      fill(0, 0, 0);
      if(gameOver) {
        textAlign(CENTER);
        text("Game over! You got " + targetsHit + " targets!", width/2, 60);
        textAlign(CENTER);
        text("Touch the green triangle to start!", width/2, 90);
      } else {
        textAlign(CENTER);
        text("Touch the green triangle to start!", width/2, 60);
      }
      tri_start.setFill(0,255,0);
    }

    world.draw();
  }
}
/* end draw section ****************************************************************************************************/


/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{

  public void showNewTarget(){
    targetsTotal++;
    float newX = random(worldWidth - 4) + 2;
    float newY = random(worldHeight - 4) + 2;
    circle1 = new FCircle(random(0.1, 2));
    circle1.setPosition(newX, newY);
    circle1.setFill(random(255), random(255), random(255));
    circle1.setStaticBody(true);
    circle1.setSensor(true);
    world.add(circle1);
    circle1Hit = false;
    world.draw();
  }

  public void run(){
    counter++;
    timer++;
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();
 
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    
    if(gameStart && !gameOver && targetsTotal - targetsHit >= gameOverDifference){
      gameStart = false;
      gameOver = true;
      world.add(tri_start);
      world.remove(box1);
      world.remove(box2);
      world.remove(box3);
      world.remove(box4);
      world.remove(circle1);
      world.remove(vicsousArea1);
      world.remove(vicsousArea2);
      b1Showing = false;
      b2Showing = false;
      b3Showing = false;
      b4Showing = false;
      v1Showing = false;
      v2Showing = false;
      box1.setPosition(10, 10);
      box2.setPosition(27, 12);
      box3.setPosition(15, 15);
      box4.setPosition(20, 5);
      s.h_avatar.setSensor(true);
    }

    if (!gameStart && s.h_avatar.isTouchingBody(tri_start)){
      targetsTotal = -1;
      targetsHit = 0;
      timer = 0;
      gameStart = true;
      gameOver = false;
      s.h_avatar.setSensor(false);
      world.remove(tri_start);
      counter = 0;
      showNewTarget();
    }

    if(gameStart && !circle1Hit && s.h_avatar.isTouchingBody(circle1)){
      targetsHit++;
      counter = 0;
      circle1Hit = true;
      world.remove(circle1);
      showNewTarget();
      
      if(!b1Showing && targetsHit == box1Timer){
        b1Showing = true;
        timeOutDelay = 4000;
        world.add(box1);
      }

      if(!b2Showing && targetsHit == box2Timer){
        b2Showing = true;
        timeOutDelay = 3000;
        world.add(box2);
      }

      if(!b3Showing && targetsHit == box3Timer){
        b3Showing = true;
        timeOutDelay = 2000;
        world.add(box3);
      }

      if(!b4Showing && targetsHit == box4Timer){
        b4Showing = true;
        timeOutDelay = 1000;
        world.add(box4);
      }

      if(!v1Showing && targetsHit == 10){
        v1Showing = true;
        world.add(vicsousArea1);
      }

      if(!v2Showing && targetsHit == 20){
        v2Showing = true;
        world.add(vicsousArea2);
      }
    }

    if(gameStart && counter >= timeOutDelay){
      println("TikTok");
      counter = 0;
      world.remove(circle1);
      showNewTarget();
    }

    if(timer % 10000 == 0){
      rotation*= -1;
    }

    if(timer % 200 == 0){
      
      if(b1Showing){
        if(targetsHit > box1Timer + 2)
          box1.setRotation(random(0,0.1) * rotation + box1.getRotation());
        if(targetsHit > box1Timer + 4)
          box1.setPosition(box1.getX() + random(-0.5, 0.5), box1.getY() + random(-0.5, 0.5));
      }
      
      if(b2Showing){
        if(targetsHit > box2Timer + 2)
          box2.setRotation(random(0,0.1) * rotation * -1 + box2.getRotation());
        if(targetsHit > box2Timer + 2)
          box2.setPosition(box2.getX() + random(-0.5, 0.5), box2.getY() + random(-0.5, 0.5));
      }

      if(b3Showing){
        if(targetsHit > box3Timer + 2)
          box3.setRotation(random(0,0.1) * rotation * -1 + box3.getRotation());
        if(targetsHit > box3Timer + 2)
          box3.setPosition(box3.getX() + random(-0.5, 0.5), box3.getY() + random(-0.5, 0.5));
      }

      if(b4Showing){
        if(targetsHit > box4Timer + 2)
          box4.setRotation(random(0,0.1) * rotation + box4.getRotation());
        if(targetsHit > box4Timer + 2)
          box4.setPosition(box4.getX() + random(-0.5, 0.5), box4.getY() + random(-0.5, 0.5));
      }
    }
  
    // /* Viscous layer codes */
    if (s.h_avatar.isTouchingBody(vicsousArea1) || s.h_avatar.isTouchingBody(vicsousArea2)){
      s.h_avatar.setDamping(500);
    }
    else {
      s.h_avatar.setDamping(10); 
    }
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/
