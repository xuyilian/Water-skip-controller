import pygame


pygame.init()
pygame.joystick.init()
global joystick
for event in pygame.event.get():
    if event.type == pygame.QUIT:
        done = True
joystick = pygame.joystick.Joystick(0)
joystick.init()
print('pygame started')
name = joystick.get_name()
print("Joystick name: {}".format(name))

PS4_Controller = {'L1': 9, 'R1': 10,
                  'Triangle': 3, 'Circle': 1, 'Square': 2, 'Cross': 0,
                  'Option': 6, 'Share': 4, 'Touchpad': 15,
                  'LeftStick': 7, 'RightStick': 8,
                  'Up': 11, 'Down': 12, 'Left': 13, 'Right': 14,
                  'L2': -4, 'R2': -5,
                  'LeftStickX': 0, 'LeftStickY': 1, 'RightStickX': 2, 'RightStickY': 3, }

KeyMapping = PS4_Controller

if name == 'PS4 Controller':
    KeyMapping = PS4_Controller
    print('Use PS4 Controller Mapping')
else:
    print('warning: no suitable Key mapping')



def step():
    global joystick
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    joystick = pygame.joystick.Joystick(0)
    joystick.init()


def quit():
    pygame.quit()
    print('pygame stopped')


def get():
    global joystick
    return joystick






def get_axis_lx():
    global joystick
    return joystick.get_axis(0)


def get_axis_ly():
    global joystick
    return joystick.get_axis(1)


def get_axis_lt():
    global joystick
    return joystick.get_axis(2)


def get_axis_rx():
    global joystick
    return joystick.get_axis(3)


def get_axis_ry():
    global joystick
    return joystick.get_axis(4)


def get_axis_rt():
    global joystick
    return joystick.get_axis(5)


def get_button_a():
    global joystick
    return joystick.get_button(0)


def get_button_b():
    global joystick
    return joystick.get_button(1)


def get_button_x():
    global joystick
    return joystick.get_button(2)


def get_button_y():
    global joystick
    return joystick.get_button(3)


def get_button_lb():
    global joystick
    return joystick.get_button(4)


def get_button_rb():
    global joystick
    return joystick.get_button(5)


def get_button_l():
    global joystick
    return joystick.get_button(6)


def get_button_r():
    global joystick
    return joystick.get_button(7)


def get_button_start():
    global joystick
    return joystick.get_button(8)


def get_button_back():
    global joystick
    return joystick.get_button(9)


def get_button_xbox():
    global joystick
    return joystick.get_button(10)


def get_button_up():
    global joystick
    return joystick.get_button(11)

def get_hat_up():
    global joystick
    return joystick.get_button(11)

def get_button_down():
    global joystick
    return joystick.get_button(12)

def get_hat_down():
    global joystick
    return joystick.get_button(12)


def get_button_left():
    global joystick
    return joystick.get_button(13)

def get_hat_left():
    global joystick
    return joystick.get_button(13)

def get_button_right():
    global joystick
    return joystick.get_button(14)

def get_hat_right():
    global joystick
    return joystick.get_button(14)

# ps 4 controller


def get_button_block():
    global joystick
    return joystick.get_button(2)


def get_button_cross():
    global joystick
    return joystick.get_button(0)


def get_button_circle():
    global joystick
    return joystick.get_button(1)


def get_button_triangle():
    global joystick
    return joystick.get_button(3)


def get_button_l1():
    global joystick
    return joystick.get_button(9)


def get_button_r1():
    global joystick
    return joystick.get_button(10)


def get_button_share():
    global joystick
    return joystick.get_button(4)


def get_button_option():
    global joystick
    return joystick.get_button(6)


def get_button_l_ps4():
    global joystick
    return joystick.get_button(7)


def get_button_r_ps4():
    global joystick
    return joystick.get_button(8)


def get_button_ps():
    global joystick
    return joystick.get_button(5)


def get_button_table():
    global joystick
    return joystick.get_button(15)


def get_hat_up_ps4():
    global joystick
    return joystick.get_button(11)

def get_hat_down_ps4():
    global joystick
    return joystick.get_button(12)


def get_hat_right_ps4():
    global joystick
    return joystick.get_button(14)


def get_hat_left_ps4():
    global joystick
    return joystick.get_button(13)


def get_axis_ry_ps4():
    global joystick
    return joystick.get_axis(3)


def get_axis_rx_ps4():
    global joystick
    return joystick.get_axis(2)


def get_axis_ly_ps4():
    global joystick
    return joystick.get_axis(1)


def get_axis_lx_ps4():
    global joystick
    return joystick.get_axis(0)


def get_axis_r2_ps4():
    global joystick
    return joystick.get_axis(5)


def get_axis_l2_ps4():
    global joystick
    return joystick.get_axis(4)


def get_power():
    global joystick
    return joystick.get_power_level()