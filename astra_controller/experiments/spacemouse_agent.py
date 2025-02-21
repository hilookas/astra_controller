from astra_controller.experiments.spacemouse_expert import SpaceMouseExpert

class SpacemouseAgent():
    def __init__(self):
        self.mouse = SpaceMouseExpert()
        
    def act(self, observation):
        action, buttons = self.mouse.get_action()
        return action, buttons
