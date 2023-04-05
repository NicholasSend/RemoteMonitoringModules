import pygame


class KeyboardController:
    """
    A class for monitoring keyboard input
    """

    def __init__(self):
        pygame.init()
        self.key_state = {}

    def read(self):
        """
        Returns a dictionary of the current state of all keys on the keyboard

        :return: dict
        """
        self.key_state = {}
        for event in pygame.event.get():
            print(event)
            if event.type == pygame.KEYDOWN:
                self.key_state[event.key] = True
            elif event.type == pygame.KEYUP:
                self.key_state[event.key] = False
        return self.key_state


if __name__ == '__main__':
    keyboard_controller = KeyboardController()

    while True:
        key_state = keyboard_controller.read()
        # print(key_state)

        # e.g. Determine if the spacebar is currently being pressed
        # if key_state.get(pygame.K_SPACE):
