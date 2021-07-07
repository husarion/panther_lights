import os

Controller = None

panther_driver_type = os.getenv('PANTHER_LIGHTS_CONTROLLER')
if panther_driver_type == 'PLT_GUI':
    try:
        import matplotlib
        import matplotlib.pyplot as plt
        import warnings
    except ImportError as e:
        raise ImportError(f'unable to import matplotlib. Make sure you have installed matplotlib. {e}')
    from .plt_gui import VirtualLEDController
    Controller = VirtualLEDController

elif panther_driver_type == 'APA102':
    try:
        import RPi.GPIO as GPIO
        from apa102_pi.driver import apa102 
    except ImportError:
        raise ImportError('no hardware specific packages installed. Make sure you are running this node on Raspberry pi.')
    from .apa102 import HardwareAPA102Controller
    Controller = HardwareAPA102Controller

else:
    raise OSError(f'\'{panther_driver_type}\' - unknown driver type')