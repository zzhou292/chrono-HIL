#include <SDL2/SDL.h>

// This program shows how to work with joysticks using SDL2.
// This example shows how to do it by manually polling the joystick
// rather than using the sdl event queue.
int main() {
  // Initialize the joystick subsystem
  SDL_Init(SDL_INIT_JOYSTICK);

  // If there are no joysticks connected, quit the program
  if (SDL_NumJoysticks() <= 0) {
    printf("There are no joysticks connected. Quitting now...\n");
    SDL_Quit();
    return -1;
  }

  // Open the joystick for reading and store its handle in the joy variable
  SDL_Joystick *joy = SDL_JoystickOpen(0);

  // If the joy variable is NULL, there was an error opening it.
  if (joy != NULL) {
    // Get information about the joystick
    const char *name = SDL_JoystickName(joy);
    const int num_axes = SDL_JoystickNumAxes(joy);
    const int num_buttons = SDL_JoystickNumButtons(joy);
    const int num_hats = SDL_JoystickNumHats(joy);

    printf("Now reading from joystick '%s' with:\n"
           "%d axes\n"
           "%d buttons\n"
           "%d hats\n\n",
           name, num_axes, num_buttons, num_hats);

    int quit = 0;

    // Keep reading the state of the joystick in a loop
    while (quit == 0) {
      if (SDL_QuitRequested()) {
        quit = 1;
      }

      for (int i = 0; i < num_axes; i++) {
        printf("Axis %d: %d\n", i, SDL_JoystickGetAxis(joy, i));
      }

      for (int i = 0; i < num_buttons; i++) {
        printf("Button %d: %d\n", i, SDL_JoystickGetButton(joy, i));
      }

      for (int i = 0; i < num_hats; i++) {
        printf("Hat %d: %d\n", i, SDL_JoystickGetHat(joy, i));
      }

      printf("\n");

      SDL_Delay(50);
    }

    SDL_JoystickClose(joy);
  } else {
    printf("Couldn't open the joystick. Quitting now...\n");
  }

  SDL_Quit();
  return 0;
}