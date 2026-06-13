import pygame


def joystick_saturation(joystick_obj):
    saturation_level = 0.15
    d_pad_z = -(joystick_obj.get_axis(3))  # get axis values
    if abs(d_pad_z) < saturation_level:
        d_pad_z = 0
    d_pad_y = (joystick_obj.get_axis(0))  # get axis values
    if abs(d_pad_y) < saturation_level:
        d_pad_y = 0
    d_pad_x = (joystick_obj.get_axis(1))  # get axis values
    if abs(d_pad_x) < saturation_level:
        d_pad_x = 0
    return d_pad_z, d_pad_y, d_pad_x


class Joystick(object):
    def __init__(self, ):
        pygame.init()
        pygame.joystick.init()
