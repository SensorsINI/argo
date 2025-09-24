import React, { useCallback, useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import {
  PanelExtensionContext,
  MessageEvent,
  ParameterValue,
} from "@foxglove/studio";
import { GeometryMsgsVector3, StdMsgsFloat64, StdMsgsBool } from "@foxglove/studio";

// Data interfaces for Argo topics
interface CompassData {
  x: number; // Magnetometer x in µT
  y: number; // Magnetometer y in µT  
  z: number; // Magnetometer z in µT (used for heading calculation)
}

interface WindData {
  x: number; // Wind speed in m/s
  y: number; // Wind angle in degrees CW from front of boat
  z: number; // Temperature in celsius
}

interface ControlData {
  x: number; // Rudder position (-1 to +1)
  y: number; // Sail position (-1 to +1)
  z: number; // Reserved
}

interface GPSVelocityData {
  x: number; // North component in knots
  y: number; // East component in knots
  z: number; // Speed magnitude in knots
}

interface ArgoSailboatState {
  compass: CompassData | null;
  wind: WindData | null;
  rudderSailRadio: ControlData | null;
  gpsVelocity: GPSVelocityData | null;
  humanControlled: boolean | null;
  timestamp: number;
}

// Convert magnetometer data to compass heading (simplified)
function magnetometerToHeading(mx: number, my: number, mz: number): number {
  // Simple 2D compass heading calculation
  // In a real implementation, you'd use proper magnetometer calibration
  let heading = Math.atan2(my, mx) * (180 / Math.PI);
  heading = (heading + 360) % 360; // Normalize to 0-360 degrees
  return heading;
}

// Convert wind angle relative to boat to absolute direction
function windToAbsoluteDirection(windAngle: number, compassHeading: number): number {
  return (compassHeading + windAngle) % 360;
}

// Main panel component
function ArgoSailboatPanel() {
  const [state, setState] = useState<ArgoSailboatState>({
    compass: null,
    wind: null,
    rudderSailRadio: null,
    gpsVelocity: null,
    humanControlled: null,
    timestamp: Date.now(),
  });

  // Calculate derived values
  const compassHeading = state.compass ? magnetometerToHeading(
    state.compass.x, 
    state.compass.y, 
    state.compass.z
  ) : null;

  const windAbsoluteDirection = state.wind && compassHeading ? 
    windToAbsoluteDirection(state.wind.y, compassHeading) : null;

  return (
    <div style={{ 
      width: "100%", 
      height: "100%", 
      display: "flex", 
      flexDirection: "column",
      fontFamily: "monospace",
      backgroundColor: "#1e1e1e",
      color: "#ffffff",
      padding: "10px"
    }}>
      <h2 style={{ margin: "0 0 20px 0", textAlign: "center", color: "#00e676" }}>
        Argo Sailboat State
      </h2>
      
      {/* Compass Heading */}
      <div style={{ marginBottom: "15px", padding: "10px", backgroundColor: "#2d2d2d", borderRadius: "5px" }}>
        <h3 style={{ margin: "0 0 10px 0", color: "#4fc3f7" }}>Compass Heading</h3>
        {compassHeading !== null ? (
          <div style={{ fontSize: "24px", fontWeight: "bold", color: "#00e676" }}>
            {compassHeading.toFixed(1)}°
          </div>
        ) : (
          <div style={{ color: "#ff5722" }}>No compass data</div>
        )}
      </div>

      {/* Wind Data */}
      <div style={{ marginBottom: "15px", padding: "10px", backgroundColor: "#2d2d2d", borderRadius: "5px" }}>
        <h3 style={{ margin: "0 0 10px 0", color: "#4fc3f7" }}>Wind Data</h3>
        {state.wind ? (
          <div>
            <div>Speed: {state.wind.x.toFixed(2)} m/s</div>
            <div>Direction (relative): {state.wind.y.toFixed(1)}°</div>
            {windAbsoluteDirection && (
              <div>Direction (absolute): {windAbsoluteDirection.toFixed(1)}°</div>
            )}
            <div>Temperature: {state.wind.z.toFixed(1)}°C</div>
          </div>
        ) : (
          <div style={{ color: "#ff5722" }}>No wind data</div>
        )}
      </div>

      {/* Control Data */}
      <div style={{ marginBottom: "15px", padding: "10px", backgroundColor: "#2d2d2d", borderRadius: "5px" }}>
        <h3 style={{ margin: "0 0 10px 0", color: "#4fc3f7" }}>Control Status</h3>
        {state.rudderSailRadio ? (
          <div>
            <div>Rudder: {state.rudderSailRadio.x.toFixed(2)} 
              <span style={{ fontSize: "12px", marginLeft: "5px" }}>
                ({state.rudderSailRadio.x > 0 ? "Right" : "Left"})
              </span>
            </div>
            <div>Sail: {state.rudderSailRadio.y.toFixed(2)}
              <span style={{ fontSize: "12px", marginLeft: "5px" }}>
                ({state.rudderSailRadio.y > 0 ? "Out" : "In"})
              </span>
            </div>
            <div>Mode: {state.humanControlled ? 
              <span style={{ color: "#ff9800" }}>Manual</span> : 
              <span style={{ color: "#00e676" }}>Autonomous</span>
            }</div>
          </div>
        ) : (
          <div style={{ color: "#ff5722" }}>No control data</div>
        )}
      </div>

      {/* GPS Velocity */}
      <div style={{ marginBottom: "15px", padding: "10px", backgroundColor: "#2d2d2d", borderRadius: "5px" }}>
        <h3 style={{ margin: "0 0 10px 0", color: "#4fc3f7" }}>GPS Velocity</h3>
        {state.gpsVelocity ? (
          <div>
            <div>Speed: {state.gpsVelocity.z.toFixed(2)} knots</div>
            <div>North: {state.gpsVelocity.x.toFixed(2)} knots</div>
            <div>East: {state.gpsVelocity.y.toFixed(2)} knots</div>
          </div>
        ) : (
          <div style={{ color: "#ff5722" }}>No GPS velocity data</div>
        )}
      </div>

      {/* Visual Indicators */}
      <div style={{ marginBottom: "15px", padding: "10px", backgroundColor: "#2d2d2d", borderRadius: "5px" }}>
        <h3 style={{ margin: "0 0 10px 0", color: "#4fc3f7" }}>Visual Indicators</h3>
        
        {/* Compass Rose */}
        <div style={{ marginBottom: "10px" }}>
          <div style={{ fontSize: "14px", marginBottom: "5px" }}>Compass Rose:</div>
          <div style={{ 
            width: "100px", 
            height: "100px", 
            border: "2px solid #4fc3f7", 
            borderRadius: "50%",
            position: "relative",
            margin: "0 auto"
          }}>
            {compassHeading !== null && (
              <div style={{
                position: "absolute",
                top: "50%",
                left: "50%",
                width: "2px",
                height: "40px",
                backgroundColor: "#00e676",
                transformOrigin: "0 0",
                transform: `translate(-1px, -40px) rotate(${compassHeading}deg)`,
                transition: "transform 0.3s ease"
              }} />
            )}
            <div style={{ position: "absolute", top: "5px", left: "50%", transform: "translateX(-50%)", fontSize: "12px" }}>N</div>
            <div style={{ position: "absolute", bottom: "5px", left: "50%", transform: "translateX(-50%)", fontSize: "12px" }}>S</div>
            <div style={{ position: "absolute", left: "5px", top: "50%", transform: "translateY(-50%)", fontSize: "12px" }}>W</div>
            <div style={{ position: "absolute", right: "5px", top: "50%", transform: "translateY(-50%)", fontSize: "12px" }}>E</div>
          </div>
        </div>

        {/* Wind Direction Arrow */}
        {windAbsoluteDirection && state.wind && (
          <div style={{ marginBottom: "10px" }}>
            <div style={{ fontSize: "14px", marginBottom: "5px" }}>Wind Direction:</div>
            <div style={{ 
              width: "100px", 
              height: "100px", 
              border: "2px solid #ff5722", 
              borderRadius: "50%",
              position: "relative",
              margin: "0 auto"
            }}>
              <div style={{
                position: "absolute",
                top: "50%",
                left: "50%",
                width: "2px",
                height: `${Math.min(40, state.wind.x * 5)}px`,
                backgroundColor: "#ff5722",
                transformOrigin: "0 0",
                transform: `translate(-1px, -${Math.min(40, state.wind.x * 5)}px) rotate(${windAbsoluteDirection}deg)`,
                transition: "transform 0.3s ease"
              }} />
            </div>
          </div>
        )}

        {/* Rudder and Sail Bars */}
        {state.rudderSailRadio && (
          <div>
            <div style={{ marginBottom: "5px" }}>
              <div style={{ fontSize: "14px", marginBottom: "2px" }}>Rudder:</div>
              <div style={{ 
                width: "100%", 
                height: "20px", 
                backgroundColor: "#333", 
                position: "relative",
                borderRadius: "10px"
              }}>
                <div style={{
                  position: "absolute",
                  left: "50%",
                  width: "4px",
                  height: "100%",
                  backgroundColor: "#00e676",
                  transform: `translateX(${state.rudderSailRadio.x * 50}%)`,
                  transition: "transform 0.3s ease"
                }} />
              </div>
            </div>
            <div>
              <div style={{ fontSize: "14px", marginBottom: "2px" }}>Sail:</div>
              <div style={{ 
                width: "100%", 
                height: "20px", 
                backgroundColor: "#333", 
                position: "relative",
                borderRadius: "10px"
              }}>
                <div style={{
                  position: "absolute",
                  left: "50%",
                  width: "4px",
                  height: "100%",
                  backgroundColor: "#4fc3f7",
                  transform: `translateX(${state.rudderSailRadio.y * 50}%)`,
                  transition: "transform 0.3s ease"
                }} />
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

export function initArgoSailboatPanel(context: PanelExtensionContext) {
  const container = context.panelElement;
  const root = createRoot(container);

  // State management
  let currentState: ArgoSailboatState = {
    compass: null,
    wind: null,
    rudderSailRadio: null,
    gpsVelocity: null,
    humanControlled: null,
    timestamp: Date.now(),
  };

  // Message handlers
  const compassHandler = useCallback((messageEvent: MessageEvent<GeometryMsgsVector3>) => {
    const msg = messageEvent.message;
    currentState.compass = { x: msg.x, y: msg.y, z: msg.z };
    currentState.timestamp = messageEvent.receiveTime.nsec / 1000000; // Convert to ms
  }, []);

  const windHandler = useCallback((messageEvent: MessageEvent<GeometryMsgsVector3>) => {
    const msg = messageEvent.message;
    currentState.wind = { x: msg.x, y: msg.y, z: msg.z };
    currentState.timestamp = messageEvent.receiveTime.nsec / 1000000;
  }, []);

  const controlHandler = useCallback((messageEvent: MessageEvent<GeometryMsgsVector3>) => {
    const msg = messageEvent.message;
    currentState.rudderSailRadio = { x: msg.x, y: msg.y, z: msg.z };
    currentState.timestamp = messageEvent.receiveTime.nsec / 1000000;
  }, []);

  const velocityHandler = useCallback((messageEvent: MessageEvent<GeometryMsgsVector3>) => {
    const msg = messageEvent.message;
    currentState.gpsVelocity = { x: msg.x, y: msg.y, z: msg.z };
    currentState.timestamp = messageEvent.receiveTime.nsec / 1000000;
  }, []);

  const humanControlHandler = useCallback((messageEvent: MessageEvent<StdMsgsBool>) => {
    currentState.humanControlled = messageEvent.message.data;
    currentState.timestamp = messageEvent.receiveTime.nsec / 1000000;
  }, []);

  // Subscribe to Argo ROS2 topics
  useEffect(() => {
    const subscriptions = [
      context.subscribe("/compass", compassHandler),
      context.subscribe("/anem_speed_angle_temp", windHandler),
      context.subscribe("/rudder_sail_radio", controlHandler),
      context.subscribe("/gps_velocity", velocityHandler),
      context.subscribe("/human_controlled", humanControlHandler),
    ];

    return () => {
      subscriptions.forEach((sub) => sub.unsubscribe());
    };
  }, [compassHandler, windHandler, controlHandler, velocityHandler, humanControlHandler]);

  // Render the panel
  const renderPanel = useCallback(() => {
    root.render(<ArgoSailboatPanel />);
  }, [root]);

  // Initial render
  renderPanel();

  // Re-render on state changes (simplified - in production you'd use proper state management)
  const interval = setInterval(renderPanel, 100); // Update every 100ms

  // Cleanup function
  return () => {
    clearInterval(interval);
    root.unmount();
  };
}

