#include <SDL2/SDL.h>
#include <cstdio>
#include <cstring>  

int main() {
  if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
    std::printf("SDL_Init failed: %s\n", SDL_GetError());
    return 1;
  }

  int num_joysticks = SDL_NumJoysticks();
  if (num_joysticks <= 0) {
    std::puts("No joysticks detected.");
    SDL_Quit();
    return 0;
  }

  std::printf("Detected %d joystick(s).\n", num_joysticks);
  // Don't open joysticks that have 'RDMCTMZT' or 'KVM' in their name
  int chosen_index = -1;
  for (int i = 0; i < num_joysticks; ++i) {
    const char *name = SDL_JoystickNameForIndex(i);
    if (!name)
      continue;

    if (std::strstr(name, "RDMCTMZT") || std::strstr(name, "KVM")) {
      continue;
    }

    // First non-keyboard/KVM device wins
    if (chosen_index == -1)
      chosen_index = i;
  }

  SDL_Joystick *joystick = SDL_JoystickOpen(chosen_index);
  if (!joystick) {
    std::printf("SDL_JoystickOpen failed: %s\n", SDL_GetError());
    SDL_Quit();
    return 1;
  }

  const char *name = SDL_JoystickName(joystick);
  int num_axes = SDL_JoystickNumAxes(joystick);
  int num_buttons = SDL_JoystickNumButtons(joystick);
  int num_hats = SDL_JoystickNumHats(joystick);

  std::printf("Joystick 0: '%s'\n", name ? name : "(unknown)");
  std::printf("  Axes: %d\n  Buttons: %d\n  Hats: %d\n\n",
              num_axes, num_buttons, num_hats);
  std::puts("Press buttons/move axes to see SDL indices. Close the window or press Ctrl+C to quit.\n");

  SDL_Event event;
  bool quit = false;
  while (!quit) {
    while (SDL_PollEvent(&event)) {
      switch (event.type) {
      case SDL_QUIT:
        quit = true;
        break;
      case SDL_JOYAXISMOTION:
        std::printf("Axis %d value %d\n", event.jaxis.axis, event.jaxis.value);
        break;
      case SDL_JOYBUTTONDOWN:
      case SDL_JOYBUTTONUP:
        std::printf("Button %d %s\n",
                    event.jbutton.button,
                    (event.type == SDL_JOYBUTTONDOWN) ? "down" : "up");
        break;
      case SDL_JOYHATMOTION:
        std::printf("Hat %d value %d\n", event.jhat.hat, event.jhat.value);
        break;
      default:
        break;
      }
    }
    SDL_Delay(5);
  }

  SDL_JoystickClose(joystick);
  SDL_Quit();
  return 0;
}
