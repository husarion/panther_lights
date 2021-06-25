from .animation import Animation
import numpy as np
import imageio
import os

class ImageAnimation(Animation):
    ANIMATION_NAME = 'image_animation'
    def __init__(self, anim_yaml, num_led, time_step, global_brightness):
        super().__init__(anim_yaml, num_led, time_step, global_brightness)

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

        if 'color' in anim_yaml.keys():
            color = anim_yaml['color']

            r = (np.uint32(color) >> 16) & (0x0000FF)
            g = (np.uint32(color) >>  8) & (0x0000FF)
            b = (np.uint32(color)      ) & (0x0000FF)
            
            # Turn image to grayscale and normalise colours
            self._img = 0.2989 * self._img[:,:,0] + \
                        0.5870 * self._img[:,:,1] + \
                        0.1140 * self._img[:,:,2]
            self._img = self._img / np.max(self._img) * 255
            img_r = (self._img.astype(np.uint16) * r / 255).astype(np.uint8)
            img_g = (self._img.astype(np.uint16) * g / 255).astype(np.uint8)
            img_b = (self._img.astype(np.uint16) * b / 255).astype(np.uint8)
            self._img = np.dstack((img_r,img_g,img_b))

        self._img = self._img.astype(np.uint16)
        r = self._img[:,:,0] << 16
        g = self._img[:,:,1] << 8
        b = self._img[:,:,2]
        self._img = r + g + b
        

        self._frame_counter = 0

        self._frame = self._img[self._frame_counter,:]
        self._frame_tics = self._max_tics / self._img_y
        self._frame_counter_threshold = 0


    def _animation_callback(self):
        if self._tick >= self._frame_counter_threshold:
            self._frame = self._img[self._frame_counter,:]
            self._frame_counter_threshold += self._frame_tics
            self._frame_counter += 1

        return self._frame


    def reset(self):
        super().reset()
        self._frame_counter = 0
        self._frame_counter_threshold = self._max_tics / self._img_y
        self._frame = self._img[self._frame_counter,:]
