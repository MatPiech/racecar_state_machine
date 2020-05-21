from statemachine import StateMachine, State, Transition

# AS - Autonomous System


class AutonomousSystemState(State):
    """
    A class used to model abstract State class for Autonomous System.

    ...

    Parameters
    ----------
    name : str
        State name.

    value : optional
        State value.

    initial : bool
        Declaration if state is initial.

    Attributes
    ----------
    TS : str
        Tractive system value.

    R2D : str
        Ready to drive value.

    SA : str
        Steering Actuator value.

    SB : str
        Service Brake value.

    EBS : str
        Emergency Brake System value.

    ASSI : str
        Autonomous System Status Indicator value.

    Methods
    -------
    set_state_parameters()
        Set state parameters according to Formula Student Autonomous System State Machine from 2020 rules.

    get_state_parameters()
        Show state parameters.
    """
    
    def __init__(self, name, value=None, initial=False):
        super(AutonomousSystemState, self).__init__(
            name=name, value=value, initial=initial)
        self.set_state_parameters()

    def set_state_parameters(self, *args, **kwargs):
        """Set state parameters according to Formula Student Autonomous System State Machine from 2020 rules."""

        self.TS = 'off'
        self.R2D = 'off'
        self.SA = 'unavailable'
        self.SB = 'unavailable'
        self.EBS = 'unavailable' 
        self.ASSI = 'off'

    def get_state_parameters(self, *args, **kwargs):
        """Show state parameters."""

        #print(f'<--------------------------------->\n{self.name} system parameters:\nTS:\t{self.TS}\nR2D:\t{self.R2D}\nSA:\t{self.SA}\nSB:\t{self.SB}\nEBS:\t{self.EBS}\nASSI:\t{self.ASSI}\n<--------------------------------->')
        print('<--------------------------------->\n{} system parameters:\nTS:\t{}\nR2D:\t{}\nSA:\t{}\nSB:\t{}\nEBS:\t{}\nASSI:\t{}\n<--------------------------------->'.format(self.name, self.TS, self.R2D, self.SA, self.SB, self.EBS, self.ASSI))


class ManualDriving(AutonomousSystemState):
    def set_state_parameters(self, *args, **kwargs):
        self.TS = 'on' 
        self.R2D = 'on'
        self.SA = 'unavailable'
        self.SB = 'unavailable'
        self.EBS = 'unavailable'
        self.ASSI = 'off'


class ASOff(AutonomousSystemState):
    def set_state_parameters(self, *args, **kwargs):
        self.TS = 'off'
        self.R2D = 'off'
        self.SA = 'unavailable'
        self.SB = 'unavailable'
        self.EBS = 'x'
        self.ASSI = 'off'


class ASReady(AutonomousSystemState):
    def set_state_parameters(self, *args, **kwargs):
        self.TS = 'on'
        self.R2D = 'off'
        self.SA = 'available'
        self.SB = 'engaged'
        self.EBS = 'armed'
        self.ASSI = 'yellow cont'


class ASDriving(AutonomousSystemState):
    def set_state_parameters(self, *args, **kwargs):
        self.TS = 'on'
        self.R2D = 'on'
        self.SA = 'available'
        self.SB = 'available'
        self.EBS = 'armed'
        self.ASSI = 'yellow flash'


class ASEmergency(AutonomousSystemState):
    def set_state_parameters(self, *args, **kwargs):
        self.TS = 'off'
        self.R2D = 'off'
        self.SA = 'x'
        self.SB = 'x'
        self.EBS = 'activated'
        self.ASSI = 'blue flash'


class ASFinished(AutonomousSystemState):
    def set_state_parameters(self, *args, **kwargs):
        self.TS = 'off'
        self.R2D = 'off'
        self.SA = 'unavailable'
        self.SB = 'x'
        self.EBS = 'activated'
        self.ASSI = 'yellow cont'


class AutonomousSystem(StateMachine):
    """
    A class used to create Autonomous System State Machine based on Formula Students 2020 rules.

    ...

    Attributes
    ----------
    transitions_map: dict
        Dictionary which map transition identifier as a key and combine with transition as a value.

    as_off : ASOff class instance

    manual_driving : ManualDriving class instance

    as_ready : ASReady class instance

    as_driving : ASDriving class instance

    as_emergency : ASEmergency class instance

    as_finished : ASFinished class instance

    off_manual : statemachine.Transition
        Transition from 'AS Off' to 'Manual Driving' state.

    manual_off : statemachine.Transition
        Transition from 'Manual Driving' to 'AS Off' state.

    off_ready : statemachine.Transition
        Transition from 'AS Off' to 'AS Ready'' state.

    ready_off : statemachine.Transition
        Transition from 'AS Ready' to 'AS Off' state.

    ready_driving : statemachine.Transition
        Transition from 'AS Ready' to 'AS Driving' state.

    ready_emergency : statemachine.Transition
        Transition from 'AS Ready' to 'AS Emergency' state.

    driving_finished : statemachine.Transition
        Transition from 'AS Driving' to 'AS Finished' state.

    driving_emergency : statemachine.Transition
        Transition from 'AS Driving' to 'AS Emergency' state.

    finished_emergency : statemachine.Transition
        Transition from 'AS Finished' to 'AS Emergency' state.

    finished_off : statemachine.Transition
        Transition from 'AS Finished' to 'AS Off' state.

    emergency_off : statemachine.Transition
        Transition from 'AS Emergency' to 'AS Off' state.

    Methods
    -------
    set_transitions_map()
        Set transition identifier as a key for transition in transitions_map dictionary.

    get_state_parameters()
        Call current state get_state_parameters method.
    """

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
        """Set transition identifier as a key for transition in transitions_map dictionary."""
        for trans in self.transitions:
            self.transitions_map[trans.identifier] = trans

    def get_state_parameters(self):
        """Call current state get_state_parameters method."""
        self.current_state.get_state_parameters()