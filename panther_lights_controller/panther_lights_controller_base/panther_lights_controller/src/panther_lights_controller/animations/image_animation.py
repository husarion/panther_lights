import numpy as np
import imageio
import os

from .animation import Animation

class ImageAnimation(Animation):

    ANIMATION_NAME = 'image_animation'
    def __init__(self, anim_yaml, num_led, global_brightness, panel):
        '''image animation class'''
        super().__init__(anim_yaml, num_led, global_brightness, panel)

        if not 'image' in anim_yaml.keys():
            raise Animation.AnimationYAMLError('no image in parameters YAML')

        img_name = anim_yaml['image']
        if not os.path.isabs(img_name):
            img_path = os.path.dirname(__file__) + f'/../../../animations/{img_name}'
        else:
            img_path = img_name
        self._img = np.array(imageio.imread(img_path))

        (img_y, img_x, _) = np.shape(self._img)
        self._img_y = img_y
        if img_x != num_led:
            raise Animation.AnimationYAMLError('supplied image is not equal width to led number')

        # overwrite animation's color
        if 'color' in anim_yaml.keys():
            color = anim_yaml['color']
            # change from hex to RGB
            r = (np.uint32(color) >> 16) & (0x0000FF)
            g = (np.uint32(color) >>  8) & (0x0000FF)
            b = (np.uint32(color)      ) & (0x0000FF)
            
            # turn image to grayscale
            self._img = 0.2989 * self._img[:,:,0] \
                      + 0.5870 * self._img[:,:,1] \
                      + 0.1140 * self._img[:,:,2]
            # normalise brightness
            self._img = self._img / np.max(self._img) * 255
            img_r = (self._img.astype(np.uint32) * r / 255).astype(np.uint8)
            img_g = (self._img.astype(np.uint32) * g / 255).astype(np.uint8)
            img_b = (self._img.astype(np.uint32) * b / 255).astype(np.uint8)
            # reconstruct image
            self._img = np.dstack((img_r,img_g,img_b))

        # conver image from RGB to HEX
        self._img = self._img.astype(np.uint32)
        r = self._img[:,:,0] << 16
        g = self._img[:,:,1] << 8
        b = self._img[:,:,2]
        self._img = r + g + b
        self._img.astype(np.uint8)
        
        self._i = 0
        self._frame_time = self._duration / self._img_y
        

    def __call__(self):
        '''returns new frame'''
        if self._i < self._img_y:
            frame = self._img[self._i,:]
            self._i += 1
            return frame
        raise Animation.AnimationFinished


    @property
    def sleep_time(self):
        '''returns time needed to sleep in thread between frames'''
        return self._frame_time


    def reset(self):
        '''restets animation to it's initial state'''
        self._i = 0

    def param(self, val):
        '''sets animations param'''
        pass
    param = property(None, param)
