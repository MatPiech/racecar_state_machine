from AutonomousSystem import AutonomousSystem as AS

SM = AS()

off_manual_off = 0
off_ready_off = 1
off_ready_emergency_off = 2
off_ready_driving_emergency_off = 3
off_ready_driving_finished_emergency_off = 4
off_ready_driving_finished_off = 5

def testing(x):
    print(SM.current_state)
    if x == 0:
        SM.manual()
        print(SM.current_state)
        SM.off1()
        print(SM.current_state)
    elif x == 1:
        SM.ready()
        print(SM.current_state)
        SM.off2()
        print(SM.current_state)
    elif x == 2:
        SM.ready()
        print(SM.current_state)
        SM.emergency1()
        print(SM.current_state)
        SM.off4()
        print(SM.current_state)
    elif x == 3:
        SM.ready()
        print(SM.current_state)
        SM.driving()
        print(SM.current_state)
        SM.emergency2()
        print(SM.current_state)
        SM.off4()
        print(SM.current_state)
    elif x == 4:
        SM.ready()
        print(SM.current_state)
        SM.driving()
        print(SM.current_state)
        SM.finished()
        print(SM.current_state)
        SM.emergency3()
        print(SM.current_state)
        SM.off4()
    elif x == 5:
        SM.ready()
        print(SM.current_state)
        SM.driving()
        print(SM.current_state)
        SM.finished()
        print(SM.current_state)
        SM.off3()
        print(SM.current_state)

#choose a route

testing(off_manual_off)


