/*
Copyright (c) 2011 Tsz-Chiu Au, Peter Stone
University of Texas at Austin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the University of Texas at Austin nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package aim4.sim;

import java.awt.Color;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Queue;
import java.util.Set;
import java.util.SortedMap;
import java.util.TreeMap;

import aim4.config.Debug;
import aim4.config.DebugPoint;
import aim4.driver.AutoDriver;
import aim4.driver.DriverSimView;
import aim4.driver.ProxyDriver;
import aim4.im.IntersectionManager;
import aim4.im.v2i.V2IManager;
import aim4.map.DataCollectionLine;
import aim4.map.BasicMap;
import aim4.map.Road;
import aim4.map.SpawnPoint;
import aim4.map.SpawnPoint.SpawnSpec;
import aim4.map.lane.Lane;
import aim4.msg.i2v.I2VMessage;
import aim4.msg.v2i.V2IMessage;
import aim4.ns3commond.CreateNode;
import aim4.ns3commond.DeleteNode;
import aim4.ns3commond.RunToTimestep;
import aim4.ns3commond.UpdatePosition;
import aim4.vehicle.AutoVehicleSimView;
import aim4.vehicle.BasicAutoVehicle;
import aim4.vehicle.HumanDrivenVehicleSimView;
import aim4.vehicle.ProxyVehicleSimView;
import aim4.vehicle.VehicleSpec;
import aim4.vehicle.VinRegistry;
import aim4.vehicle.VehicleSimView;
import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.ServerSocket;
//add by weixin 
import java.net.Socket;
import java.net.SocketException;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;


/**
 * The autonomous drivers only simulator.
 */
public class AutoDriverOnlySimulator implements Simulator {

  /////////////////////////////////
  // NESTED CLASSES
  /////////////////////////////////
    private enum msgType {
    t_V2IMsg,
    t_I2VMsg,
  };
  /**
   * The result of a simulation step.
   */
  public static class AutoDriverOnlySimStepResult implements Simulator.SimStepResult {

    /** The VIN of the completed vehicles in this time step */
    List<Integer> completedVINs;

    /**
     * Create a result of a simulation step
     *
     * @param completedVINs  the VINs of completed vehicles.
     */
    public AutoDriverOnlySimStepResult(List<Integer> completedVINs) {
      this.completedVINs = completedVINs;
    }

    /**
     * Get the list of VINs of completed vehicles.
     *
     * @return the list of VINs of completed vehicles.
     */
    public List<Integer> getCompletedVINs() {
      return completedVINs;
    }
  }

  /////////////////////////////////
  // PRIVATE FIELDS
  /////////////////////////////////

  /** The map */
  private BasicMap basicMap;
  /** All active vehicles, in form of a map from VINs to vehicle objects. */
  private Map<Integer,VehicleSimView> vinToVehicles;
  /** The current time */
  private double currentTime;
  /** The number of completed vehicles */
  private int numOfCompletedVehicles;
  /** The total number of bits transmitted by the completed vehicles */
  private int totalBitsTransmittedByCompletedVehicles;
  /** The total number of bits received by the completed vehicles */
  private int totalBitsReceivedByCompletedVehicles;

  private Map<Integer,VehicleSimView> preStepVehicles;  /*add by weixin*/
  private List<Integer> completedVINsrecord;
  private Map<Integer,I2VMessage> NSAdapterI2VMsgMap;
  private Map<Integer,V2IMessage> NSAdapterV2IMsgMap;
  /////////////////////////////////
  // CLASS CONSTRUCTORS
  /////////////////////////////////

  /**
   * Create an instance of the simulator.
   *
   * @param basicMap             the map of the simulation
   */
  public AutoDriverOnlySimulator(BasicMap basicMap) {
    this.basicMap = basicMap;
    this.vinToVehicles = new HashMap<Integer,VehicleSimView>();
      
    this.preStepVehicles = new HashMap<Integer,VehicleSimView>();/*add by weixin*/
    this.completedVINsrecord = new LinkedList<Integer>();
    this.NSAdapterI2VMsgMap = new HashMap<Integer,I2VMessage>();
    this.NSAdapterV2IMsgMap = new HashMap<Integer,V2IMessage>();
    
    currentTime = 0.0;
    numOfCompletedVehicles = 0;
    totalBitsTransmittedByCompletedVehicles = 0;
    totalBitsReceivedByCompletedVehicles = 0;
  }

  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  // the main loop

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized AutoDriverOnlySimulator.AutoDriverOnlySimStepResult step(double timeStep) {
    System.err.printf("------------------%.2f start--------------------\n",this.getSimulationTime());//test by weixin
    if (Debug.PRINT_SIMULATOR_STAGE) {
      System.err.printf("--------------------------------------\n");
      System.err.printf("------SIM:spawnVehicles---------------\n");
    }
    spawnVehicles(timeStep);
    if (Debug.PRINT_SIMULATOR_STAGE) {
      System.err.printf("------SIM:provideSensorInput---------------\n");
    }
    provideSensorInput();
    if (Debug.PRINT_SIMULATOR_STAGE) {
      System.err.printf("------SIM:letDriversAct---------------\n");
    }
    letDriversAct();
    if (Debug.PRINT_SIMULATOR_STAGE) {
      System.err.printf("------SIM:letIntersectionManagersAct--------------\n");
    }
    letIntersectionManagersAct(timeStep);
    if (Debug.PRINT_SIMULATOR_STAGE) {
      System.err.printf("------SIM:communication---------------\n");
    }
        try {
            communication(timeStep);
        } catch (IOException ex) {
            Logger.getLogger(AutoDriverOnlySimulator.class.getName()).log(Level.SEVERE, null, ex);
        }
    if (Debug.PRINT_SIMULATOR_STAGE) {
      System.err.printf("------SIM:moveVehicles---------------\n");
    }
    moveVehicles(timeStep);
    if (Debug.PRINT_SIMULATOR_STAGE) {
      System.err.printf("------SIM:cleanUpCompletedVehicles---------------\n");
    }
    
    //List<Integer> completedVINs = cleanUpCompletedVehicles();//delete by weixin 
    completedVINsrecord.clear();
    completedVINsrecord.addAll(cleanUpCompletedVehicles());//change by weixin
//add by weixin
    System.err.printf("------------------%.2f end--------------------\n",this.getSimulationTime());//test by weixin 
    preStepVehicles.clear();
    preStepVehicles.putAll(vinToVehicles);
//weixin add end   
    currentTime += timeStep;
    // debug
    checkClocks();
    //change by weixin
    return new AutoDriverOnlySimulator.AutoDriverOnlySimStepResult(completedVINsrecord);
    //return new AutoDriverOnlySimStepResult(completedVINs);
  }

  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  // information retrieval

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized BasicMap getMap() {
    return basicMap;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized double getSimulationTime() {
    return currentTime;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized int getNumCompletedVehicles() {
    return numOfCompletedVehicles;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized double getAvgBitsTransmittedByCompletedVehicles() {
    if (numOfCompletedVehicles > 0) {
      return ((double)totalBitsTransmittedByCompletedVehicles)
             / numOfCompletedVehicles;
    } else {
      return 0.0;
    }
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized double getAvgBitsReceivedByCompletedVehicles() {
    if (numOfCompletedVehicles > 0) {
      return ((double)totalBitsReceivedByCompletedVehicles)
             / numOfCompletedVehicles;
    } else {
      return 0.0;
    }
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized Set<VehicleSimView> getActiveVehicles() {
    return new HashSet<VehicleSimView>(vinToVehicles.values());
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public VehicleSimView getActiveVehicle(int vin) {
    return vinToVehicles.get(vin);
  }


  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized void addProxyVehicle(ProxyVehicleSimView vehicle) {
    Point2D pos = vehicle.getPosition();
    Lane minLane = null;
    double minDistance = -1.0;

    for(Road road : basicMap.getRoads()) {
      for(Lane lane : road.getLanes()) {
        double d = lane.nearestDistance(pos);
        if (minLane == null || d < minDistance) {
          minLane = lane;
          minDistance = d;
        }
      }
    }
    assert minLane != null;

    ProxyDriver driver = vehicle.getDriver();
    if (driver != null) {
      driver.setCurrentLane(minLane);
      driver.setSpawnPoint(null);
      driver.setDestination(null);
    }

    vinToVehicles.put(vehicle.getVIN(), vehicle);
  }


  /////////////////////////////////
  // PRIVATE METHODS
  /////////////////////////////////

  /////////////////////////////////
  // STEP 1
  /////////////////////////////////

  /**
   * Spawn vehicles.
   *
   * @param timeStep  the time step
   */
  private void spawnVehicles(double timeStep) {
    for(SpawnPoint spawnPoint : basicMap.getSpawnPoints()) {
      List<SpawnSpec> spawnSpecs = spawnPoint.act(timeStep);
      if (!spawnSpecs.isEmpty()) {
        if (canSpawnVehicle(spawnPoint)) {
          for(SpawnSpec spawnSpec : spawnSpecs) {
            VehicleSimView vehicle = makeVehicle(spawnPoint, spawnSpec);
            VinRegistry.registerVehicle(vehicle); // Get vehicle a VIN number
            vinToVehicles.put(vehicle.getVIN(), vehicle);
            break; // only handle the first spawn vehicle
                   // TODO: need to fix this
          }
        } // else ignore the spawnSpecs and do nothing
      }
    }
  }


  /**
   * Whether a spawn point can spawn any vehicle
   *
   * @param spawnPoint  the spawn point
   * @return Whether the spawn point can spawn any vehicle
   */
  private boolean canSpawnVehicle(SpawnPoint spawnPoint) {
    // TODO: can be made much faster.
    Rectangle2D noVehicleZone = spawnPoint.getNoVehicleZone();
    for(VehicleSimView vehicle : vinToVehicles.values()) {
      if (vehicle.getShape().intersects(noVehicleZone)) {
        return false;
      }
    }
    return true;
  }

  /**
   * Create a vehicle at a spawn point.
   *
   * @param spawnPoint  the spawn point
   * @param spawnSpec   the spawn specification
   * @return the vehicle
   */
  private VehicleSimView makeVehicle(SpawnPoint spawnPoint,
                                     SpawnSpec spawnSpec) {
    VehicleSpec spec = spawnSpec.getVehicleSpec();
    Lane lane = spawnPoint.getLane();
    // Now just take the minimum of the max velocity of the vehicle, and
    // the speed limit in the lane
    double initVelocity = Math.min(spec.getMaxVelocity(), lane.getSpeedLimit());
    // Obtain a Vehicle
    AutoVehicleSimView vehicle =
      new BasicAutoVehicle(spec,
                           spawnPoint.getPosition(),
                           spawnPoint.getHeading(),
                           spawnPoint.getSteeringAngle(),
                           initVelocity, // velocity
                           initVelocity,  // target velocity
                           spawnPoint.getAcceleration(),
                           spawnSpec.getSpawnTime());
    // Set the driver
    AutoDriver driver = new AutoDriver(vehicle, basicMap);
    driver.setCurrentLane(lane);
    driver.setSpawnPoint(spawnPoint);
    driver.setDestination(spawnSpec.getDestinationRoad());
    vehicle.setDriver(driver);

    return vehicle;
  }


  /////////////////////////////////
  // STEP 2
  /////////////////////////////////

  /**
   * Compute the lists of vehicles of all lanes.
   *
   * @return a mapping from lanes to lists of vehicles sorted by their
   *         distance on their lanes
   */
  private Map<Lane,SortedMap<Double,VehicleSimView>> computeVehicleLists() {
    // Set up the structure that will hold all the Vehicles as they are
    // currently ordered in the Lanes
    Map<Lane,SortedMap<Double,VehicleSimView>> vehicleLists =
      new HashMap<Lane,SortedMap<Double,VehicleSimView>>();
    for(Road road : basicMap.getRoads()) {
      for(Lane lane : road.getLanes()) {
        vehicleLists.put(lane, new TreeMap<Double,VehicleSimView>());
      }
    }
    // Now add each of the Vehicles, but make sure to exclude those that are
    // already inside (partially or entirely) the intersection
    for(VehicleSimView vehicle : vinToVehicles.values()) {
      // Find out what lanes it is in.
      Set<Lane> lanes = vehicle.getDriver().getCurrentlyOccupiedLanes();
      for(Lane lane : lanes) {
        // Find out what IntersectionManager is coming up for this vehicle
        IntersectionManager im =
          lane.getLaneIM().nextIntersectionManager(vehicle.getPosition());
        // Only include this Vehicle if it is not in the intersection.
        if(lane.getLaneIM().distanceToNextIntersection(vehicle.getPosition())>0
            || im == null || !im.intersects(vehicle.getShape().getBounds2D())) {
          // Now find how far along the lane it is.
          double dst = lane.distanceAlongLane(vehicle.getPosition());
          // Now add it to the map.
          vehicleLists.get(lane).put(dst, vehicle);
        }
      }
    }
    // Now consolidate the lists based on lanes
    for(Road road : basicMap.getRoads()) {
      for(Lane lane : road.getLanes()) {
        // We may have already removed this Lane from the map
        if(vehicleLists.containsKey(lane)) {
          Lane currLane = lane;
          // Now run through the lanes
          while(currLane.hasNextLane()) {
            currLane = currLane.getNextLane();
            // Put everything from the next lane into the original lane
            // and remove the mapping for the next lane
            vehicleLists.get(lane).putAll(vehicleLists.remove(currLane));
          }
        }
      }
    }

    return vehicleLists;
  }

  /**
   * Compute the next vehicles of all vehicles.
   *
   * @param vehicleLists  a mapping from lanes to lists of vehicles sorted by
   *                      their distance on their lanes
   * @return a mapping from vehicles to next vehicles
   */
  private Map<VehicleSimView, VehicleSimView> computeNextVehicle(
    Map<Lane,SortedMap<Double,VehicleSimView>> vehicleLists) {
    // At this point we should only have mappings for start Lanes, and they
    // should include all the Lanes they run into.  Now we need to turn this
    // into a hash map that maps Vehicles to the next vehicle in the Lane
    // or any Lane the Lane runs into
    Map<VehicleSimView, VehicleSimView> nextVehicle =
      new HashMap<VehicleSimView,VehicleSimView>();
    // For each of the ordered lists of vehicles
    for(SortedMap<Double,VehicleSimView> vehicleList : vehicleLists.values()) {
      VehicleSimView lastVehicle = null;
      // Go through the Vehicles in order of their position in the Lane
      for(VehicleSimView currVehicle : vehicleList.values()) {
        if(lastVehicle != null) {
          // Create the mapping from the previous Vehicle to the current one
          nextVehicle.put(lastVehicle,currVehicle);
        }
        lastVehicle = currVehicle;
      }
    }

    return nextVehicle;
  }

  /**
   * Provide each vehicle with sensor information to allow it to make
   * decisions.  This works first by making an ordered list for each Lane of
   * all the vehicles in that Lane, in order from the start of the Lane to
   * the end of the Lane.  We must make sure to leave out all vehicles that
   * are in the intersection.  We must also concatenate the lists for lanes
   * that feed into one another.  Then, for each vehicle, depending on the
   * state of its sensors, we provide it with the appropriate sensor input.
   */
  private void provideSensorInput() {
    Map<Lane,SortedMap<Double,VehicleSimView>> vehicleLists =
      computeVehicleLists();
    Map<VehicleSimView, VehicleSimView> nextVehicle =
      computeNextVehicle(vehicleLists);

    provideIntervalInfo(nextVehicle);
    provideVehicleTrackingInfo(vehicleLists);
    provideTrafficSignal();
  }

  /**
   * Provide sensing information to the intervalometers of all vehicles.
   *
   * @param nextVehicle  a mapping from vehicles to next vehicles
   */
  private void provideIntervalInfo(
    Map<VehicleSimView, VehicleSimView> nextVehicle) {

    // Now that we have this list set up, let's provide input to all the
    // Vehicles.
    for(VehicleSimView vehicle: vinToVehicles.values()) {
      // If the vehicle is autonomous
      if (vehicle instanceof AutoVehicleSimView) {
        AutoVehicleSimView autoVehicle = (AutoVehicleSimView)vehicle;

        switch(autoVehicle.getLRFMode()) {
        case DISABLED:
          // Find the interval to the next vehicle
          double interval;
          // If there is a next vehicle, then calculate it
          if(nextVehicle.containsKey(autoVehicle)) {
            // It's the distance from the front of this Vehicle to the point
            // at the rear of the Vehicle in front of it
            interval = calcInterval(autoVehicle, nextVehicle.get(autoVehicle));
          } else { // Otherwise, just set it to the maximum possible value
            interval = Double.MAX_VALUE;
          }
          // Now actually record it in the vehicle
          autoVehicle.getIntervalometer().record(interval);
          autoVehicle.setLRFSensing(false); // Vehicle is not using
                                            // the LRF sensor
          break;
        case LIMITED:
          // FIXME
          autoVehicle.setLRFSensing(true); // Vehicle is using the LRF sensor
          break;
        case ENABLED:
          // FIXME
          autoVehicle.setLRFSensing(true); // Vehicle is using the LRF sensor
          break;
        default:
          throw new RuntimeException("Unknown LRF Mode: " +
                                     autoVehicle.getLRFMode().toString());
        }
      }
    }
  }

  /**
   * Provide tracking information to vehicles.
   *
   * @param vehicleLists  a mapping from lanes to lists of vehicles sorted by
   *                      their distance on their lanes
   */
  private void provideVehicleTrackingInfo(
    Map<Lane, SortedMap<Double, VehicleSimView>> vehicleLists) {
    // Vehicle Tracking
    for(VehicleSimView vehicle: vinToVehicles.values()) {
      // If the vehicle is autonomous
      if (vehicle instanceof AutoVehicleSimView) {
        AutoVehicleSimView autoVehicle = (AutoVehicleSimView)vehicle;

        if (autoVehicle.isVehicleTracking()) {
          DriverSimView driver = autoVehicle.getDriver();
          Lane targetLane = autoVehicle.getTargetLaneForVehicleTracking();
          Point2D pos = autoVehicle.getPosition();
          double dst = targetLane.distanceAlongLane(pos);

          // initialize the distances to infinity
          double frontDst = Double.MAX_VALUE;
          double rearDst = Double.MAX_VALUE;
          VehicleSimView frontVehicle = null ;
          VehicleSimView rearVehicle = null ;

          // only consider the vehicles on the target lane
          SortedMap<Double,VehicleSimView> vehiclesOnTargetLane =
            vehicleLists.get(targetLane);

          // compute the distances and the corresponding vehicles
          try {
            double d = vehiclesOnTargetLane.tailMap(dst).firstKey();
            frontVehicle = vehiclesOnTargetLane.get(d);
            frontDst = (d-dst)-frontVehicle.getSpec().getLength();
          } catch(NoSuchElementException e) {
            frontDst = Double.MAX_VALUE;
            frontVehicle = null;
          }
          try {
            double d = vehiclesOnTargetLane.headMap(dst).lastKey();
            rearVehicle = vehiclesOnTargetLane.get(d);
            rearDst = dst-d;
          } catch(NoSuchElementException e) {
            rearDst = Double.MAX_VALUE;
            rearVehicle = null;
          }

          // assign the sensor readings

          autoVehicle.getFrontVehicleDistanceSensor().record(frontDst);
          autoVehicle.getRearVehicleDistanceSensor().record(rearDst);

          // assign the vehicles' velocities

          if(frontVehicle!=null) {
            autoVehicle.getFrontVehicleSpeedSensor().record(
                frontVehicle.getVelocity());
          } else {
            autoVehicle.getFrontVehicleSpeedSensor().record(Double.MAX_VALUE);
          }
          if(rearVehicle!=null) {
            autoVehicle.getRearVehicleSpeedSensor().record(
                rearVehicle.getVelocity());
          } else {
            autoVehicle.getRearVehicleSpeedSensor().record(Double.MAX_VALUE);
          }

          // show the section on the viewer
          if (Debug.isTargetVIN(driver.getVehicle().getVIN())) {
            Point2D p1 = targetLane.getPointAtNormalizedDistance(
                Math.max((dst-rearDst)/targetLane.getLength(),0.0));
            Point2D p2 = targetLane.getPointAtNormalizedDistance(
                Math.min((frontDst+dst)/targetLane.getLength(),1.0));
            Debug.addLongTermDebugPoint(
              new DebugPoint(p2, p1, "cl", Color.RED.brighter()));
          }
        }
      }
    }

  }

  /**
   * Provide traffic signals.
   */
  private void provideTrafficSignal() {
    for(VehicleSimView vehicle: vinToVehicles.values()) {
      if (vehicle instanceof HumanDrivenVehicleSimView) {
        HumanDrivenVehicleSimView manualVehicle =
          (HumanDrivenVehicleSimView)vehicle;
        provideTrafficLightSignal(manualVehicle);
      }
    }
  }

  /**
   * Calculate the distance between vehicle and the next vehicle on a lane.
   *
   * @param vehicle      the vehicle
   * @param nextVehicle  the next vehicle
   * @return the distance between vehicle and the next vehicle on a lane
   */
  private double calcInterval(VehicleSimView vehicle,
                              VehicleSimView nextVehicle) {
    // From Chiu: Kurt, if you think this function is not okay, probably
    // we should talk to see what to do.
    Point2D pos = vehicle.getPosition();
    if(nextVehicle.getShape().contains(pos)) {
      return 0.0;
    } else {
      // TODO: make it more efficient
      double interval = Double.MAX_VALUE ;
      for(Line2D edge : nextVehicle.getEdges()) {
        double dst = edge.ptSegDist(pos);
        if(dst < interval){
          interval = dst;
        }
      }
      return interval;
    }
  }
  // Kurt's code:
  // interval = vehicle.getPosition().
  //   distance(nextVehicle.get(vehicle).getPointAtRear());


  /**
   * Provide traffic light signals to a vehicle.
   *
   * @param vehicle  the vehicle
   */
  private void provideTrafficLightSignal(HumanDrivenVehicleSimView vehicle) {
    // TODO: implement it later
//    DriverSimView driver = vehicle.getDriver();
//    Lane currentLane = driver.getCurrentLane();
//    Point2D pos = vehicle.getPosition();
//    IntersectionManager im = currentLane.getLaneIM().
//                             nextIntersectionManager(pos);
//    if (im != null) {
//      if (im instanceof LightOnlyManager) {
//        LightOnlyManager lightOnlyIM = (LightOnlyManager)im;
//        if (!im.getIntersection().getArea().contains(pos)) {
//          LightState s = lightOnlyIM.getLightState(currentLane);
//          vehicle.setLightState(s);
//          if (driver instanceof HumanDriver) {
//            ((HumanDriver)driver).setLightState(s);
//          }
//        } else {
//          vehicle.setLightState(null);
//          if (driver instanceof HumanDriver) {
//            ((HumanDriver)driver).setLightState(null);
//          }
//        }
//      }
//    } else {
//      vehicle.setLightState(null);
//      if (driver instanceof HumanDriver) {
//        ((HumanDriver)driver).setLightState(null);
//      }
//    }
  }

  /////////////////////////////////
  // STEP 3
  /////////////////////////////////

  /**
   * Allow each driver to act.
   */
  private void letDriversAct() {
    for(VehicleSimView vehicle : vinToVehicles.values()) {
      vehicle.getDriver().act();
    }
  }

  /////////////////////////////////
  // STEP 4
  /////////////////////////////////

  /**
   * Allow each intersection manager to act.
   *
   * @param timeStep  the time step
   */
  private void letIntersectionManagersAct(double timeStep) {
    for(IntersectionManager im : basicMap.getIntersectionManagers()) {
      im.act(timeStep);
    }
  }

  /////////////////////////////////
  // STEP 5
  /////////////////////////////////

  /**
   * Deliver the V2I and I2V messages.
   */
  private void communication(double timestep)
           throws IOException
  {

      /*************** used for connect ns3******************
            int nsport=3333;
            String nsIP="127.0.0.1";
             
            Socket connNS3Socket= new Socket(nsIP,nsport);
            deliverNewAddNodes(connNS3Socket);
            deliverDeleteNodes(connNS3Socket);
            deliverVehicleDistribution(connNS3Socket);
            runOneNs3Step(connNS3Socket,(int)timestep);

            connNS3Socket.close();
            System.err.printf("finish sending packet to NS3");
             ******************************************/
            //deliverV2IMessages2(dos);
            //deliverI2VMessages2(dos);
            
            deliverV2IMessages();
            deliverI2VMessages();
            
             

//    deliverV2VMessages();
  }
  private void runOneNs3Step(Socket socket,int timestep)
          throws IOException  
  {
      RunToTimestep rtt = new RunToTimestep((int)this.currentTime+timestep);
      boolean status = rtt.sendtoNS3(socket);
      while(status!=true)
          {
              status=rtt.sendtoNS3(socket);
          }
      System.err.printf("run one ns3 step success!\n");
      
  }
  private void deliverNewAddNodes(Socket socket)
          throws IOException  
  {
      Map<Integer,VehicleSimView> newAddVehicles= new HashMap<Integer,VehicleSimView>();
      for(int vin:vinToVehicles.keySet())
      {
          if(!preStepVehicles.containsKey(vin))//vin is new added
          {
               newAddVehicles.put(vin, vinToVehicles.get(vin));
          }
      }
      System.err.printf("-----------------start to print new added vehicles at %.2f-------------------\n",getSimulationTime());
      
      for(VehicleSimView av: newAddVehicles.values() )
      {
          System.err.printf("add vehicle %d\n",av.getVIN());
          CreateNode newaddnode=new CreateNode((float)av.getPosition().getX(),(float)av.getPosition().getY(),av.getVIN());
          boolean status=newaddnode.sendtoNS3(socket);
          while(status!=true)
          {
              status=newaddnode.sendtoNS3(socket);
          }
          System.err.printf("add vehicle %d success!\n",av.getVIN());
      }
  }
  private void deliverDeleteNodes(Socket socket)
          throws IOException  
  {
      System.err.printf("-----------------start to print completed vehicles at %.2f-------------------\n",getSimulationTime());
       List<Integer> CoVe=this.completedVINsrecord;
       DeleteNode deletenodes = new DeleteNode(CoVe);
       boolean status = deletenodes.sendtoNS3(socket);
       while(status!=true)
       {
           status = deletenodes.sendtoNS3(socket);
       }
       for(int cv: CoVe )
      {
          System.err.printf("delete vehicle %dsuccess\n",cv);
      }
  }
  private DataOutputStream deliverV2IMessages2(DataOutputStream dos)
          throws IOException 
  {
        for(VehicleSimView vehicle : vinToVehicles.values()) {
      // Start with V2I messages
            if (vehicle instanceof AutoVehicleSimView) {
                AutoVehicleSimView sender = (AutoVehicleSimView)vehicle;
                Queue<V2IMessage> v2iOutbox = sender.getV2IOutbox();
                while(!v2iOutbox.isEmpty()) {
                    V2IMessage msg = v2iOutbox.poll();
                    V2IManager receiver =
                    (V2IManager)basicMap.getImRegistry().get(msg.getImId());

                    int msgIdentifier =new Random().nextInt(100000);
                    while(NSAdapterV2IMsgMap.keySet().contains(msgIdentifier))
                    {
                        msgIdentifier = new Random().nextInt(100000);
                    }          
                    NSAdapterV2IMsgMap.put(msgIdentifier, msg);
              //如果收到来自ns3回应该消息已经收到，则删除该包；
              //如果n久之后都没有回应，则超过一定时间界限就删除该包，需要进一步参考AIM内部消息处理代码
                    AutoDriverOnlySimulator.msgType mt=AutoDriverOnlySimulator.msgType.t_V2IMsg;
                    dos.writeInt(mt.ordinal());
                    dos.writeInt(msgIdentifier);
                    dos.writeInt(sender.getVIN());
                    dos.writeInt(receiver.getId());
          
                }
            }
        }
        return dos;
  }
  private DataOutputStream deliverI2VMessages2(DataOutputStream dos)
          throws IOException 
  {
      for(IntersectionManager im : basicMap.getIntersectionManagers()) 
      {
            V2IManager senderIM = (V2IManager)im;
            for(Iterator<I2VMessage> i2vIter = senderIM.outboxIterator();
            i2vIter.hasNext();) 
            {
                I2VMessage msg = i2vIter.next();
                AutoVehicleSimView vehicle =
                (AutoVehicleSimView)VinRegistry.getVehicleFromVIN(
                msg.getVin());
                
                int msgIdentifier =new Random().nextInt(100000);
                while(NSAdapterI2VMsgMap.keySet().contains(msgIdentifier))
                {
                    msgIdentifier = new Random().nextInt(100000);
                }          
                NSAdapterI2VMsgMap.put(msgIdentifier, msg);
              //如果收到来自ns3回应该消息已经收到，则删除该包；
              //如果n久之后都没有回应，则超过一定时间界限就删除该包，需要进一步参考AIM内部消息处理代码
                AutoDriverOnlySimulator.msgType mt=AutoDriverOnlySimulator.msgType.t_I2VMsg;
                dos.writeInt(mt.ordinal());
                dos.writeInt(msgIdentifier);
                dos.writeInt(senderIM.getId());
                dos.writeInt(vehicle.getVIN());
            }
      // Done delivering the IntersectionManager's messages, so clear the
      // outbox.
            senderIM.clearOutbox();
       }
        return dos;
  }
   /**
   * Deliver Vehicle Distribution data to NS3.
   * Author:weixin
   */
  private void deliverVehicleDistribution(Socket socket)
       throws IOException   
  {
    
      System.err.printf("-----------------start to print active vehicles at %.2f-------------------\n",getSimulationTime());
      for(VehicleSimView vehicle : vinToVehicles.values())
      {
          Point2D location= vehicle.getPosition();
          UpdatePosition updatenode = new UpdatePosition((float)vehicle.getPosition().getX(),(float)vehicle.getPosition().getY(),vehicle.getVIN());
          boolean status = updatenode.sendtoNS3(socket);
          while(status!=true)
          {
              status=updatenode.sendtoNS3(socket);
          }
          System.err.printf("update vehicle %d position success:X->%.2f\tY->%.2f\n",vehicle.getVIN(),location.getX(),location.getY());       
      }
  }
   

  /**
   * Deliver the V2I messages.
   */
  private void deliverV2IMessages() {
    // Go through each vehicle and deliver each of its messages
    for(VehicleSimView vehicle : vinToVehicles.values()) {
      // Start with V2I messages
      if (vehicle instanceof AutoVehicleSimView) {
        AutoVehicleSimView sender = (AutoVehicleSimView)vehicle;
        Queue<V2IMessage> v2iOutbox = sender.getV2IOutbox();
        while(!v2iOutbox.isEmpty()) {
          V2IMessage msg = v2iOutbox.poll();
          
          
          V2IManager receiver =
            (V2IManager)basicMap.getImRegistry().get(msg.getImId());
          
          // Calculate the distance the message must travel
          double txDistance =
            sender.getPosition().distance(
                receiver.getIntersection().getCentroid());
          // Find out if the message will make it that far
          if(transmit(txDistance, sender.getTransmissionPower())) {
            // Actually deliver the message
            receiver.receive(msg);
            // Add the delivery to the debugging information
          }
          // Either way, we increment the number of transmitted messages
        }
      }
    }
  }

  /**
   * Deliver the I2V messages.
   */
  private void deliverI2VMessages() {
    // Now deliver all the I2V messages
    for(IntersectionManager im : basicMap.getIntersectionManagers()) {
      V2IManager senderIM = (V2IManager)im;
      for(Iterator<I2VMessage> i2vIter = senderIM.outboxIterator();
          i2vIter.hasNext();) {
        I2VMessage msg = i2vIter.next();
        AutoVehicleSimView vehicle =
          (AutoVehicleSimView)VinRegistry.getVehicleFromVIN(
            msg.getVin());
        // Calculate the distance the message must travel
        double txDistance =
          senderIM.getIntersection().getCentroid().distance(
            vehicle.getPosition());
        // Find out if the message will make it that far
        if(transmit(txDistance, senderIM.getTransmissionPower())) {
          // Actually deliver the message
          vehicle.receive(msg);
        }
      }
      // Done delivering the IntersectionManager's messages, so clear the
      // outbox.
      senderIM.clearOutbox();
    }
  }

//  private void deliverV2VMessages() {
//
//    // Create a place to store broadcast messages until they can be sent so
//    // that we don't have to run through the list of Vehicles for each one
//    List<V2VMessage> broadcastMessages = new ArrayList<V2VMessage>();
//
//    // Go through each vehicle and deliver each of its messages
//    for(VehicleSimView vehicle : vinToVehicles.values()) {
//      // Then do V2V messages.
//      if (vehicle instanceof AutoVehicleSimView) {
//        AutoVehicleSimView sender = (AutoVehicleSimView)vehicle;
//        for(V2VMessage msg : sender.getV2VOutbox()) {
//          if(msg.isBroadcast()) {
//            // If it's a broadcast message, we save it and worry about it later
//            broadcastMessages.add(msg);
//          } else {
//            // Otherwise, we just deliver it! Woo!
//            // TODO: need to check whether the vehicle is AutoVehicleSpec
//            AutoVehicleSimView receiver =
//              (AutoVehicleSimView)VinRegistry.getVehicleFromVIN(
//                msg.getDestinationID());
//            // Calculate the distance the message must travel
//            double txDistance =
//              sender.getPosition().distance(receiver.getPosition());
//            // Find out if the message will make it that far
//            if(transmit(txDistance, sender.getTransmissionPower())) {
//              // Actually deliver the message
//              receiver.receive(msg);
//              // Add the delivery to the debugging information
//            }
//          }
//        }
//        // Done delivering the V2V messages (except broadcast which we will
//        // handle in a moment), so clear the outbox
//        sender.getV2VOutbox().clear();
//      }
//    }
//    // Now go through the vehicles and deliver the broadcast messages
//    for(V2VMessage msg : broadcastMessages) {
//      // Send a copy to the IM for debugging/statistics purposes
//      IntersectionManager im =
//        basicMap.getImRegistry().get(msg.getIntersectionManagerID());
////      if(im != null) {
////        switch(im.getIntersectionType()) {
////        case V2V:
////          ((V2VManager) im).logBroadcast(msg);
////        }
////      }
//      // Determine who sent this message
//      // TODO: need to check whether the vehicle is AutoVehicleSpec
//      AutoVehicleSimView sender =
//        (AutoVehicleSimView)VinRegistry.getVehicleFromVIN(
//          msg.getSourceID());
//      // Deliver to each vehicle
//      for(VehicleSimView vehicle : vinToVehicles.values()) {
//        if (vehicle instanceof AutoVehicleSimView) {
//          AutoVehicleSimView receiver = (AutoVehicleSimView)vehicle;
//          // Except the one that sent it
//          if(sender != receiver) {
//            // Find out how far away they are
//            double txDistance =
//              sender.getPosition().distance(receiver.getPosition());
//            // See if this Vehicle is close enough to receive the message
//            if(transmit(txDistance, sender.getTransmissionPower())) {
//              // Actually deliver the message
//              receiver.receive(msg);
//            }
//          }
//        } // else ignore other non-autonomous vehicle
//      }
//    }
//  }


  /**
   * Whether the transmission of a message is successful
   *
   * @param distance  the distance of the transmission
   * @param power     the power of the transmission
   * @return whether the transmission of a messsage is successful
   */
  private boolean transmit(double distance, double power) {
    // Simple for now
    return distance <= power;
  }


  /////////////////////////////////
  // STEP 6
  /////////////////////////////////

  /**
   * Move all the vehicles.
   *
   * @param timeStep  the time step
   */
  private void moveVehicles(double timeStep) {
    for(VehicleSimView vehicle : vinToVehicles.values()) {
      Point2D p1 = vehicle.getPosition();
      vehicle.move(timeStep);
      Point2D p2 = vehicle.getPosition();
      for(DataCollectionLine line : basicMap.getDataCollectionLines()) {
        line.intersect(vehicle, currentTime, p1, p2);
      }
      if (Debug.isPrintVehicleStateOfVIN(vehicle.getVIN())) {
        vehicle.printState();
      }
    }
  }


  /////////////////////////////////
  // STEP 7
  /////////////////////////////////

  /**
   * Remove all completed vehicles.
   *
   * @return the VINs of the completed vehicles
   */
  private List<Integer> cleanUpCompletedVehicles() {
    List<Integer> completedVINs = new LinkedList<Integer>();

    Rectangle2D mapBoundary = basicMap.getDimensions();
    
    //System.err.printf("mapBoundary width %.2f\n",mapBoundary.getWidth());
    //System.err.printf("mapBoundary Height %.2f\n",mapBoundary.getHeight());
    
    List<Integer> removedVINs = new ArrayList<Integer>(vinToVehicles.size());
    for(int vin : vinToVehicles.keySet()) {
      VehicleSimView v = vinToVehicles.get(vin);
      // If the vehicle is no longer in the layout
      // TODO: this should be replaced with destination zone.
      if(!v.getShape().intersects(mapBoundary)) {
          //System.err.printf("going to delete vehicle %d position is (%.2f,%.2f)\n",v.getVIN(),v.getPosition().getX(),v.getPosition().getY());
        // Process all the things we need to from this vehicle
        if (v instanceof AutoVehicleSimView) {
          AutoVehicleSimView v2 = (AutoVehicleSimView)v;
          totalBitsTransmittedByCompletedVehicles += v2.getBitsTransmitted();
          totalBitsReceivedByCompletedVehicles += v2.getBitsReceived();
        }
        removedVINs.add(vin);
      }
    }
    // Remove the marked vehicles
    for(int vin : removedVINs) {
      vinToVehicles.remove(vin);
      completedVINs.add(vin);
      numOfCompletedVehicles++;
    }

    return completedVINs;
  }

  /////////////////////////////////
  // DEBUG
  /////////////////////////////////

  /**
   * Check whether the clocks are in sync.
   */
  private void checkClocks() {
    // Check the clocks for all autonomous vehicles.
    for(VehicleSimView vehicle: vinToVehicles.values()) {
      vehicle.checkCurrentTime(currentTime);
    }
    // Check the clocks for all the intersection managers.
    for(IntersectionManager im : basicMap.getIntersectionManagers()) {
      im.checkCurrentTime(currentTime);
    }
  }


}
