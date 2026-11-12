within Vehicles.Microduck;
package Sensors "Inertial processing; RGB and ToF are rendered by Three.js"
  function specificForce "Accelerometer at an offset: acceleration minus gravity"
    input Real worldToBody[3,3];
    input Real originAcceleration[3] "World coordinates";
    input Real angularVelocity[3] "Body coordinates";
    input Real angularAcceleration[3] "Body coordinates";
    input Real offset[3] "Body coordinates";
    input Real gravity[3] = {0,0,-9.80665};
    output Real value[3];
  algorithm
    value := worldToBody*(originAcceleration-gravity)
      +cross(angularAcceleration,offset)+cross(angularVelocity,cross(angularVelocity,offset));
  end specificForce;
end Sensors;
