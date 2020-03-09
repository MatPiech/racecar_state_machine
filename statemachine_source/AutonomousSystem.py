from statemachine import StateMachine, State

# AS - Autonomous System

class AutonomousSystemState(State):
    def __init__(self, name, value=None, initial=False):
        # type: (Text, Optional[V], bool) -> None
        self.name = name
        self.value = value
        self._initial = initial
        self.identifier = None      # type: Optional[Text]
        self.transitions = []       # type: List[Transition]
        self.set_system_parameters()
        
    def set_system_parameters(self, *args, **kwargs):
        # Autonomous system parameters
        self.TS = 'off'             # Tractive system
        self.R2D = 'off'            # Ready to drive
        self.SA = 'unavailable'     # Steering Actuator
        self.SB = 'unavailable'     # Service Brake
        self.EBS = 'unavailable'    # Emergency Brake System
        self.ASSI = 'off'           # Autonomous System Status Indicator

class ManualDriving(AutonomousSystemState):
    def set_system_parameters(self, *args, **kwargs):
        # Autonomous system parameters
        self.TS = 'on'              # Tractive system
        self.R2D = 'on'             # Ready to drive
        self.SA = 'unavailable'     # Steering Actuator
        self.SB = 'unavailable'     # Service Brake
        self.EBS = 'unavailable'    # Emergency Brake System
        self.ASSI = 'off'           # Autonomous System Status Indicator

class ASOff(AutonomousSystemState):
    def set_system_parameters(self, *args, **kwargs):
        # Autonomous system parameters
        self.TS = 'off'             # Tractive system
        self.R2D = 'off'            # Ready to drive
        self.SA = 'unavailable'     # Steering Actuator
        self.SB = 'unavailable'     # Service Brake
        self.EBS = 'x'              # Emergency Brake System
        self.ASSI = 'off'           # Autonomous System Status Indicator

class ASReady(AutonomousSystemState):
    def set_system_parameters(self, *args, **kwargs):
        # Autonomous system parameters
        self.TS = 'on'              # Tractive system
        self.R2D = 'off'            # Ready to drive
        self.SA = 'available'       # Steering Actuator
        self.SB = 'engaged'         # Service Brake
        self.EBS = 'armed'          # Emergency Brake System
        self.ASSI = 'yellow cont'   # Autonomous System Status Indicator

class ASDriving(AutonomousSystemState):
    def set_system_parameters(self, *args, **kwargs):
        # Autonomous system parameters
        self.TS = 'on'              # Tractive system
        self.R2D = 'on'             # Ready to drive
        self.SA = 'available'       # Steering Actuator
        self.SB = 'available'       # Service Brake
        self.EBS = 'armed'          # Emergency Brake System
        self.ASSI = 'yellow flash'  # Autonomous System Status Indicator

class ASEmergency(AutonomousSystemState):
    def set_system_parameters(self, *args, **kwargs):
        # Autonomous system parameters
        self.TS = 'off'             # Tractive system
        self.R2D = 'off'            # Ready to drive
        self.SA = 'x'               # Steering Actuator
        self.SB = 'x'               # Service Brake
        self.EBS = 'activated'      # Emergency Brake System
        self.ASSI = 'blue flash'    # Autonomous System Status Indicator

class ASFinished(AutonomousSystemState):
    def set_system_parameters(self, *args, **kwargs):
        # Autonomous system parameters
        self.TS = 'off'             # Tractive system
        self.R2D = 'off'            # Ready to drive
        self.SA = 'unavailable'     # Steering Actuator
        self.SB = 'x'               # Service Brake
        self.EBS = 'activated'      # Emergency Brake System
        self.ASSI = 'yellow cont'   # Autonomous System Status Indicator  

class AutonomousSystem(StateMachine):
    # Autonomous System States initialization
    as_off = ASOff('AS Off', initial=True)
    manual_driving = ManualDriving('Manual Driving')
    as_ready = ASReady('AS Ready')
    as_driving = ASDriving('AS Driving')
    as_emergency = ASEmergency('AS Emergency')
    as_finished = ASFinished('AS Finished')
    
    # Autonomous System Transitions
    manual = as_off.to(manual_driving)
    off1 = manual_driving.to(as_off)

    ready = as_off.to(as_ready)
    off2 = as_ready.to(as_off)
    
    driving = as_ready.to(as_driving)
    emergency1 = as_ready.to(as_emergency)
    
    finished = as_driving.to(as_finished)
    emergency2 = as_driving.to(as_emergency)
    
    emergency2 = as_finished.to(as_emergency)
    off3 = as_finished.to(as_off)
    
    off4 = as_emergency.to(as_off)