# State Machine Controller Implementation

## Branch Management

Create feature branch `controller_state_machine` from current `argo-base-node-refactor` branch. The branch will be designed for clean merge back to master via rebase (keeping commit history intact). No complex changes to other files, ensuring merge will be straightforward.

## State Machine Architecture

### Design Pattern: Hierarchical State Machine

Use a hierarchical state machine with:

- **Top-level states**: Separate classes inheriting from `BaseState`

- **Sub-states**: Enum or nested classes within sailing states

- **State context**: BoatState + additional state-specific data

- **Transition table**: Dictionary mapping (state, condition) → next_state

- **Event-driven + periodic**: Hybrid approach with sensor callbacks updating context, periodic evaluation in control loop

### State Class Hierarchy

```python

class BaseState(ABC):

    """Base class for all states"""

    def __init__(self, config): ...

    

    @abstractmethod

    def enter(self, context): 

        """Called when entering this state"""

        

    @abstractmethod  

    def update(self, context) -> ControlCommand:

        """Called each control loop iteration"""

        

    @abstractmethod

    def evaluate_transitions(self, context) -> Optional[str]:

        """Return name of next state, or None to stay"""

        

    def exit(self, context):

        """Called when leaving this state (optional override)"""

```

### Top-Level States (New Classes)

1. **InitialState**: Startup state, waits for valid sensor data

2. **GoToSailingAreaState**: Navigate to designated sailing area  

3. **PatrollingState**: Active sailing within area (contains sub-states)

4. **StuckRescueMeState**: Detected stuck condition, request help

5. **BecalmedState**: No wind condition, drift mode

### Sailing Sub-States (Within PatrollingState)

- **UpwindState**: Sailing into wind, includes tacking logic

- **DownwindState**: Sailing with wind

- **BroadReachState**: Sailing across wind

- **HoveToState**: Stationary sailing maneuver

### Maneuvers (Actions within States)

Maneuvers like tacking/jibing are implemented as multi-step actions within sailing states:

```python

class UpwindState(BaseState):

    def __init__(self, config):

        self.tacking = False

        self.tack_start_time = None

        self.tack_phase = 0  # 0=not tacking, 1=turn through wind, 2=stabilize

```

## Implementation Details

### File: `nodes/controller.py` - State Machine Integration

**Preserve existing functionality**:

- Keep all sensor callbacks unchanged

- Keep BoatState dataclass (add state_machine_context field)

- Keep service handlers (pause, switch_controller)

- Keep data collection logic

**Add state machine framework**:

- Define `BaseState` abstract class

- Define `StateMachine` class managing current state and transitions

- Define state classes (InitialState, GoToSailingAreaState, etc.)

- Define transition criteria functions

**Modify ControllerNode**:

- Add `self.state_machine = StateMachine(config)`

- In `timer_callback()`, replace controller logic with:
  ```python
  
  if not self.is_paused():
  
      control_command = self.state_machine.update(self.boat_state)
  
      # Publish command as before
  
  ```


### Transition Criteria Examples

**Simple conditions**:

- GPS distance thresholds for go_to_sailing_area → patrolling

- Wind speed threshold for patrolling → becalmed

- Time without movement for patrolling → stuck_rescue_me

**Complex conditions**:

- Wind angle + heading + speed fusion for sub-state selection

- Hysteresis on wind angle to prevent oscillation between states

- Time-based windows for maneuver completion

### State Persistence

**Restart behavior**: On controller restart:

1. Always enter InitialState first

2. InitialState evaluates conditions and transitions intelligently:

            - If inside sailing area with valid sensors → PatrollingState

            - If outside sailing area → GoToSailingAreaState  

            - If no GPS fix → wait in InitialState

No persistent state files needed - intelligent restart based on current conditions.

## Testing Strategy

### Simulation Testing (Manual)

**Setup**:

1. Use lifecycle manager: `python3 launch/argo_lifecycle_manager.py simulate`

2. Simulator provides: `/pose`, `/gps_cog`, `/gps_sog`, `/anem_speed_angle_temp`, etc.

3. Controller operates in simulation mode with state machine

**Test scenarios**:

- **State transitions**: Observe state changes via `/controller_state` topic

- **Sub-state behavior**: Monitor control commands during sailing states

- **Maneuvers**: Test tacking by setting upwind course

- **Edge cases**: Test becalmed (low wind), stuck (no movement), boundary exits

**Visualization in Foxglove**:

- Add Text panel showing `/controller_state` topic

- Plot panel showing `/controller_state_info` (custom Vector3 with state metadata)

- Map panel showing boat position relative to sailing area boundaries

### Automated Testing (Unit Tests)

Create `tests/test_state_machine.py`:

- Mock BoatState with controllable sensor values

- Test each state's `evaluate_transitions()` with various inputs

- Test state entry/exit callbacks

- Test maneuver logic within states

- Verify control command generation in each state

**Test execution**:

```bash

python3 -m pytest tests/test_state_machine.py -v

```

## Files to Create/Modify

### New Files

- `nodes/state_machine.py` - State machine framework (BaseState, StateMachine, all state classes)

- `tests/test_state_machine.py` - Unit tests for state machine

- `config/state_machine_config.yaml` - State machine configuration (thresholds, timeouts)

### Modified Files  

- `nodes/controller.py`:

        - Import state_machine module

        - Replace controller logic with state machine

        - Add state machine initialization

        - Publish state info to new topics

        - (~200 line changes, mostly additions)

### Integration with Existing Code

**Preserve compatibility**:

- All existing ROS2 topics remain unchanged

- All sensor callbacks unchanged  

- All services (pause, switch_controller) unchanged

- Data collection functionality unchanged

- Can run alongside existing controller types initially

**New topics**:

- `/controller_state` (String) - Already exists, repurpose for state name

- `/controller_state_info` (Vector3) - x=top_level_state_id, y=sub_state_id, z=time_in_state

## State Machine Configuration

`config/state_machine_config.yaml`:

```yaml

state_machine:

  # Distance thresholds (nautical miles)

  sailing_area_entry_distance: 0.1  # Enter sailing area when within 0.1nm

  sailing_area_exit_distance: 0.2   # Exit sailing area when beyond 0.2nm

  

  # Wind thresholds

  becalmed_wind_speed: 1.0          # m/s, below this is becalmed

  becalmed_timeout: 300             # seconds, wait before declaring becalmed

  

  # Stuck detection

  stuck_speed_threshold: 0.1        # knots, below this is stuck

  stuck_timeout: 180                # seconds, wait before declaring stuck

  

  # Sailing angles (degrees relative to wind)

  upwind_angle_max: 50              # 0-50° is upwind

  broad_reach_angle_min: 50         # 50-130° is broad reach  

  broad_reach_angle_max: 130

  downwind_angle_min: 130           # 130-180° is downwind

  

  # Tacking parameters

  tack_angle_through_wind: 90       # degrees to turn through wind

  tack_stabilization_time: 5        # seconds to stabilize after tack

  

  # Jibing parameters  

  jibe_angle_through_wind: 30       # degrees to turn through wind

  jibe_stabilization_time: 3        # seconds to stabilize after jibe

```

## Merge Strategy

**Clean merge preparation**:

1. Regular commits on `controller_state_machine` branch with clear messages

2. Keep changes focused on controller state machine only

3. Before merge: rebase onto latest master to resolve conflicts early

4. Merge to master via: `git rebase master` then fast-forward merge

**No merge conflicts expected because**:

- Changes isolated to controller.py and new files

- No modifications to other nodes or launch scripts  

- Preserves all existing interfaces and topics

- Can coexist with existing controller types

## Documentation Updates

Add to controller.py header comments:

- State machine architecture overview

- State descriptions and transition criteria

- Configuration parameter documentation

- Testing instructions for simulation mode