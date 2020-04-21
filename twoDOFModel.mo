model twoDOFModel

  import SI = Modelica.SIunits;
  
  parameter SI.Mass massVeh = 725 "Mass of the vehicle";
  parameter SI.Inertia Izz = 1150 "Izz Total of the vehicle";
  parameter SI.Distance wheelBase = 3.69 "Wheelbase of vehicle";
  parameter Real cgLocation = 0.46 "Distance from front axle";
  parameter Real cf = 10 "Front Cornering Stiffness";
  parameter Real cr = 10 "Rear Cornering Stiffness";
  parameter SI.Distance wheelRadius = 0.326 "Wheel Radius Rear";
    
  SI.Distance a = wheelBase*cgLocation;
  SI.Distance b = wheelBase*(1-cgLocation);
  SI.Velocity vx, vy, vx0 = 5, vy0 = 0;
  SI.Torque driveTorque, brakeTorque;
  Real alphaf, alphar, omegaz, beta;
  Real pathX, pathY;
  Real delta;
//  Real curvature;
//  Real Ybeta, Yr, Ydelta;
//  Real Nbeta, Nr, Ndelta; 
  
  SI.Distance pathDistance;
  
initial equation
  vx = vx0;
  vy = vy0;
  
equation

  driveTorque = 5;
  brakeTorque = 0;

  delta = if time >=3 then 5 else 0;
  
  alphaf = (vy + omegaz * a) / vx - delta;
  alphar = (vy + omegaz * b) / vx;
  
// Lateral Force Balance
  cf * alphaf + cr * alphar = massVeh * (omegaz * vx + der(vy));
// Yaw moment Balance
  a * cf * alphaf - b * cr * alphar = Izz * der(omegaz);
// Longitudinal Forces
  massVeh * der(vx) = vy * omegaz + driveTorque/wheelRadius - brakeTorque/wheelRadius;
// Beta
  beta = atan(vy / vx);
// Vehicle Path
  der(pathX) = vx;
  der(pathY) = vy;
  pathDistance = (pathX^2+pathY^2)^0.5;

// New bit before i break it  - need to sort out inital condition...
//  if abs(vy) < 1e-5 then
//    curvature = 0;
//  else
//    curvature = vx^2/der(vy);
//  end if;
    
// Control and Stability Derivatives
//  Ybeta = cf + cr;
//  Yr = 1 / vx * (a * cf - b * cr);
//  Ydelta = -cf;
//  Nbeta = a * cf - b * cr;
//  Nr = 1 / vx * (a ^ 2 * cf + b ^ 2 * cr);
//  Ndelta = -a * cf;
  annotation(
    uses(Modelica(version = "3.2.2")));

end twoDOFModel;
