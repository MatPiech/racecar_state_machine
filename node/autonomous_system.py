from statemachine import StateMachine, State, Transition

# AS - Autonomous System


class AutonomousSystemState(State):
    def __init__(self, name, value=None, initial=False):
        # type: (Text, Optional[V], bool) -> None
        super(AutonomousSystemState, self).__init__(
            name=name, value=value, initial=initial)
        self.set_state_parameters()

    def set_state_parameters(self, *args, **kwargs):
        # Autonomous system parameters
        self.TS = 'off'             # Tractive system
        self.R2D = 'off'            # Ready to drive
        self.SA = 'unavailable'     # Steering Actuator
        self.SB = 'unavailable'     # Service Brake
        self.EBS = 'unavailable'    # Emergency Brake System
        self.ASSI = 'off'           # Autonomous System Status Indicator

    def get_state_parameters(self, *args, **kwargs):
        #print(f'<--------------------------------->\n{self.name} system parameters:\nTS:\t{self.TS}\nR2D:\t{self.R2D}\nSA:\t{self.SA}\nSB:\t{self.SB}\nEBS:\t{self.EBS}\nASSI:\t{self.ASSI}\n<--------------------------------->')
        print('<--------------------------------->\n{} system parameters:\nTS:\t{}\nR2D:\t{}\nSA:\t{}\nSB:\t{}\nEBS:\t{}\nASSI:\t{}\n<--------------------------------->'.format(self.name, self.TS, self.R2D, self.SA, self.SB, self.EBS, self.ASSI))


class ManualDriving(AutonomousSystemState):
    def set_state_parameters(self, *args, **kwargs):
        # Autonomous system parameters
        self.TS = 'on'              # Tractive system
        self.R2D = 'on'             # Ready to drive
        self.SA = 'unavailable'     # Steering Actuator
        self.SB = 'unavailable'     # Service Brake
        self.EBS = 'unavailable'    # Emergency Brake System
        self.ASSI = 'off'           # Autonomous System Status Indicator


class ASOff(AutonomousSystemState):
    def set_state_parameters(self, *args, **kwargs):
        # Autonomous system parameters
        self.TS = 'off'             # Tractive system
        self.R2D = 'off'            # Ready to drive
        self.SA = 'unavailable'     # Steering Actuator
        self.SB = 'unavailable'     # Service Brake
        self.EBS = 'x'              # Emergency Brake System
        self.ASSI = 'off'           # Autonomous System Status Indicator


class ASReady(AutonomousSystemState):
    def set_state_parameters(self, *args, **kwargs):
        # Autonomous system parameters
        self.TS = 'on'              # Tractive system
        self.R2D = 'off'            # Ready to drive
        self.SA = 'available'       # Steering Actuator
        self.SB = 'engaged'         # Service Brake
        self.EBS = 'armed'          # Emergency Brake System
        self.ASSI = 'yellow cont'   # Autonomous System Status Indicator


class ASDriving(AutonomousSystemState):
    def set_state_parameters(self, *args, **kwargs):
        # Autonomous system parameters
        self.TS = 'on'              # Tractive system
        self.R2D = 'on'             # Ready to drive
        self.SA = 'available'       # Steering Actuator
        self.SB = 'available'       # Service Brake
        self.EBS = 'armed'          # Emergency Brake System
        self.ASSI = 'yellow flash'  # Autonomous System Status Indicator


class ASEmergency(AutonomousSystemState):
    def set_state_parameters(self, *args, **kwargs):
        # Autonomous system parameters
        self.TS = 'off'             # Tractive system
        self.R2D = 'off'            # Ready to drive
        self.SA = 'x'               # Steering Actuator
        self.SB = 'x'               # Service Brake
        self.EBS = 'activated'      # Emergency Brake System
        self.ASSI = 'blue flash'    # Autonomous System Status Indicator


class ASFinished(AutonomousSystemState):
    def set_state_parameters(self, *args, **kwargs):
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
    off_manual = as_off.to(manual_driving)
    manual_off = manual_driving.to(as_off)
    off_ready = as_off.to(as_ready)
    ready_off = as_ready.to(as_off)
    ready_driving = as_ready.to(as_driving)
    ready_emergency = as_ready.to(as_emergency)
    driving_finished = as_driving.to(as_finished)
    driving_emergency = as_driving.to(as_emergency)
    finished_emergency = as_finished.to(as_emergency)
    finished_off = as_finished.to(as_off)
    emergency_off = as_emergency.to(as_off)

    # initialize AutonomousSystem object with StateMachine __init__ method
    def __init__(self):
        super(AutonomousSystem, self).__init__()
        self.transitions_map = {}
        self.set_transitions_map()

    # define a printable autonomous system state machine representation
    def __repr__(self):
        return "{}(model={!r}, state_field={!r}, current_state={!r})".format(
            type(self).__name__, self.model, self.state_field, self.current_state.identifier)

    def set_transitions_map(self):
        for trans in self.transitions:
            self.transitions_map[trans.identifier] = trans

    def get_state_parameters(self):
        self.current_state.get_state_parameters()