class StateHandler:
    def __init__(self):
        pass

    def match_state(self, state):
        match state:
            case "Driving":
                pass
            case "Aiming_Driving":
                pass
            case "Shooting_Driving":
                print("PLS DONT SHOOT")
            case "Aligning":
                print("Aligning pls wait")
            case _:
                pass