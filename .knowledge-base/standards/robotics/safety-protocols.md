---
title: "Safety Protocols for Robotics Work"
category: "robotics"
tags: ["safety", "lipo", "e-stop", "testing", "protocols", "hardware", "humanoid"]
description: "Safety standards for robotics development including battery handling, testing zones, emergency procedures, and software safety mechanisms"
last_updated: "2026-03-18"
---

# Safety Protocols for Robotics Work

> **Philosophy:** Safety is non-negotiable. A moving robot is a hazard. A LiPo battery is a fire risk. Every team member MUST read this document before touching hardware. When in doubt, power off.

## LiPo Battery Handling

### Charging Rules (MANDATORY)

| Rule | Requirement |
|------|-------------|
| Charger | Use balance charger ONLY (charges each cell equally) |
| Charging rate | 1C or less (e.g., 2200mAh battery → charge at 2.2A max) |
| Supervision | NEVER leave charging batteries unattended |
| Surface | Charge on fireproof surface (LiPo bag, ceramic tile, metal tray) |
| Location | Away from flammable materials, near fire extinguisher |
| Voltage check | Verify cell voltages before charging (no cell below 3.0V) |
| Temperature | Do not charge if battery feels hot or is swollen |

### Storage Rules

| Rule | Requirement |
|------|-------------|
| Storage voltage | 3.7-3.85V per cell (use storage charge mode on charger) |
| Temperature | Room temperature (15-25C), away from direct sunlight |
| Container | Fireproof LiPo bag or ammo can with vent holes |
| Duration | Check voltage monthly if stored long-term |
| Location | Away from exits, not blocking evacuation routes |

### Voltage Reference

| State | Voltage per Cell | Action |
|-------|-----------------|--------|
| Fully charged | 4.20V | Ready for use |
| Storage charge | 3.80V | For long-term storage |
| Nominal | 3.70V | Normal operating range |
| Low warning | 3.50V | Return robot, stop testing |
| Critical | 3.30V | Land/stop immediately |
| Damaged | < 3.00V | DO NOT charge — dispose safely |

### Disposal

- Discharge to 0V using a LiPo discharger or low-wattage resistor
- Submerge in salt water for 24 hours (1 tablespoon salt per cup of water)
- Wrap in tape, dispose at electronics recycling facility
- NEVER put LiPo batteries in regular trash

### Fire Response

1. **DO NOT** use water on a LiPo fire (chemical reaction produces hydrogen)
2. Use a Class D fire extinguisher, sand, or dry chemical extinguisher
3. If small and contained: let it burn out in a safe location outdoors
4. Evacuate if smoke is heavy — LiPo fumes are toxic
5. Call emergency services for uncontrolled fires

## Robot Testing Safety Zones

### Zone Setup

```
┌──────────────────────────────────────────────┐
│                                              │
│   ┌──────────────────────────────────────┐   │
│   │          SAFETY ZONE (2m+)           │   │
│   │                                      │   │
│   │   ┌──────────────────────────────┐   │   │
│   │   │      TEST AREA               │   │   │
│   │   │                              │   │   │
│   │   │      [Robot]                 │   │   │
│   │   │                              │   │   │
│   │   └──────────────────────────────┘   │   │
│   │                                      │   │
│   │          OPERATOR STATION            │   │
│   │          [E-Stop within reach]       │   │
│   └──────────────────────────────────────┘   │
│                                              │
│   Bystander exclusion zone                   │
└──────────────────────────────────────────────┘
```

| Zone | Distance | Requirement |
|------|----------|-------------|
| Test area | Robot operating region | Clear of obstacles, no fragile items |
| Safety zone | 2m around test area | Only operator and designated observers |
| Exclusion zone | Beyond safety zone | No bystanders during active testing |

### Testing Environment Rules

- Clear the test area of cables, loose objects, and trip hazards
- Ensure good lighting (robot cameras need it, operators need visibility)
- Mark the safety zone with tape or cones for longer test sessions
- Inform nearby people that testing is in progress
- Keep a clear path to the exit at all times

## Emergency Stop Procedures

### Hardware E-Stop (MANDATORY for All Mobile Robots)

Every mobile robot MUST have a physical emergency stop button that:
- Cuts power to ALL motors immediately
- Is colored RED and clearly labeled
- Is accessible without reaching over the robot
- Does NOT require software to function (hardware kill switch)

### Software E-Stop

```python
"""Software emergency stop implementation."""

class EmergencyStopNode(Node):
    """Monitors for e-stop conditions and halts the robot."""

    def __init__(self):
        super().__init__('emergency_stop')
        self._estop_active = False

        # Subscribe to e-stop trigger
        self.create_subscription(
            Bool, '/e_stop', self._estop_callback, 10
        )

        # Override cmd_vel when e-stop active
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._override_timer = self.create_timer(0.05, self._enforce_stop)

    def _estop_callback(self, msg: Bool) -> None:
        if msg.data and not self._estop_active:
            self.get_logger().fatal('E-STOP ACTIVATED')
        self._estop_active = msg.data

    def _enforce_stop(self) -> None:
        if self._estop_active:
            self._cmd_pub.publish(Twist())  # Zero velocity
```

### E-Stop Procedure

1. **Press hardware E-stop** (always first)
2. Verify robot has stopped moving
3. If software-only: run `ros2 topic pub /e_stop std_msgs/msg/Bool "data: true" --once`
4. Investigate the cause before resuming
5. Reset hardware E-stop, verify safe conditions, then resume

## Weight and Speed Limits for Testing

| Environment | Max Weight | Max Linear Speed | Max Angular Speed |
|-------------|-----------|-----------------|-------------------|
| Indoor (office/lab) | 15 kg | 0.5 m/s | 1.0 rad/s |
| Indoor (warehouse) | 30 kg | 1.0 m/s | 1.5 rad/s |
| Outdoor (controlled) | 50 kg | 2.0 m/s | 2.0 rad/s |
| Inspection/confined space | 10 kg | 0.3 m/s | 0.5 rad/s |

**Enforce in software:**

```python
# In motor controller node — MANDATORY velocity clamping
MAX_LINEAR = 0.5   # m/s — adjust per environment
MAX_ANGULAR = 1.0  # rad/s

def _cmd_callback(self, msg: Twist) -> None:
    clamped_linear = max(-MAX_LINEAR, min(MAX_LINEAR, msg.linear.x))
    clamped_angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, msg.angular.z))
    self._controller.set_velocity(clamped_linear, clamped_angular)
```

## Personal Protective Equipment

| Situation | Required PPE |
|-----------|-------------|
| General robot testing | Safety glasses |
| LiPo charging/handling | Safety glasses, fire-resistant gloves |
| Soldering/hardware assembly | Safety glasses, anti-static wrist strap |
| Outdoor testing | Closed-toe shoes, high-visibility vest |
| Working with heavy robots (>15kg) | Steel-toe shoes, gloves |

## Software Safety Mechanisms

### Velocity Limits (MANDATORY)

Every motor controller MUST enforce maximum velocity limits in software, independent of any upstream node.

### Watchdog Timer (MANDATORY)

Motors MUST stop if no command is received within timeout. See [hardware-integration.md](hardware-integration.md) for implementation.

### Geofencing

For autonomous operation, implement virtual boundaries:

```python
class GeofenceNode(Node):
    """Prevent robot from leaving designated test area."""

    def __init__(self):
        super().__init__('geofence')
        self.declare_parameter('boundary_radius', 5.0)  # meters from origin
        self._boundary = self.get_parameter('boundary_radius').value

        self.create_subscription(Odometry, '/odom', self._check_boundary, 10)
        self._estop_pub = self.create_publisher(Bool, '/e_stop', 10)

    def _check_boundary(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        distance = (x**2 + y**2) ** 0.5

        if distance > self._boundary:
            self.get_logger().fatal(f'GEOFENCE BREACH: distance={distance:.2f}m')
            self._estop_pub.publish(Bool(data=True))
```

### Speed Ramp-Up

Never go from zero to max speed instantly. Always ramp:

```python
ACCEL_LIMIT = 0.5  # m/s^2

def _apply_ramp(self, target: float, current: float, dt: float) -> float:
    max_change = ACCEL_LIMIT * dt
    if target > current:
        return min(target, current + max_change)
    return max(target, current - max_change)
```

## Humanoid Robot Specific Safety

> Applicable when working with humanoid platforms (future work).

| Rule | Requirement |
|------|-------------|
| Fall protection | Test with robot suspended/tethered until stable walking proven |
| Joint limits | Software AND hardware joint limits on all actuators |
| Speed ramp-up | Start at 10% speed, incrementally increase |
| Arm reach | No humans within arm reach during autonomous operation |
| Head/camera movement | Limit speed to prevent whiplash-like motions |
| Standing tests | Use safety harness attached to overhead support |

## Incident Reporting Procedure

### When to Report

Report ALL incidents, including near-misses:
- Robot collision with person or property
- Battery swelling, overheating, or fire
- Uncontrolled robot movement
- E-stop failure
- Sensor failure leading to dangerous behavior
- Near-misses (robot came close to causing harm)

### Report Template

```
INCIDENT REPORT
Date/Time: ___________
Location: ___________
Personnel present: ___________
Robot/equipment involved: ___________

What happened:
___________

Root cause (if known):
___________

Immediate actions taken:
___________

Proposed preventive measures:
___________

Was anyone injured: YES / NO
Was property damaged: YES / NO
Was the e-stop used: YES / NO
```

File incident reports in the project repository: `docs/incidents/YYYY-MM-DD_description.md`

## Safety Checklist Before Each Test Session

### Pre-Test (MANDATORY — do not skip)

- [ ] Battery voltage checked (above 3.5V per cell)
- [ ] Battery not swollen or damaged
- [ ] Hardware E-stop tested and functional
- [ ] Software E-stop tested (`/e_stop` topic works)
- [ ] Velocity limits configured for test environment
- [ ] Watchdog timer active on motor controller
- [ ] Test area cleared of obstacles and people
- [ ] Safety zone established and communicated
- [ ] Fire extinguisher accessible
- [ ] At least one other person aware testing is occurring
- [ ] ros2 bag recording started (for incident review)

### Post-Test

- [ ] Robot powered off
- [ ] Battery put to storage charge if not testing again today
- [ ] Batteries stored in LiPo bags
- [ ] Test area cleaned up
- [ ] Any incidents or near-misses documented
- [ ] Key findings recorded in experiment notes

## Lab/Workspace Setup Requirements

| Requirement | Details |
|-------------|---------|
| Fire extinguisher | Class ABC or Class D, within 5m of charging station |
| First aid kit | Basic kit accessible in the lab |
| Ventilation | Adequate for LiPo fume evacuation |
| Power | Sufficient outlets, no daisy-chained power strips for chargers |
| Storage | Fireproof container for LiPo batteries |
| Floor | Clear of cables and trip hazards |
| Lighting | Sufficient for robot cameras and operator visibility |
| Signage | "Robot Testing in Progress" sign for active sessions |
| Emergency contacts | Posted visibly (building security, fire department) |

## Quick Checklist

- [ ] All team members have read this document before touching hardware
- [ ] LiPo batteries handled per charging/storage/disposal rules
- [ ] Hardware E-stop installed and tested on every mobile robot
- [ ] Software velocity limits enforced in motor controller
- [ ] Watchdog timer stops motors on command timeout
- [ ] Safety zone established for every test session
- [ ] Pre-test checklist completed before EVERY session
- [ ] Incident reporting procedure understood by all team members
- [ ] Fire extinguisher accessible near charging station
- [ ] Geofencing enabled for autonomous navigation tests
