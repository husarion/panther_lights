from .animation import Animation

from .slide import Slide
from .image_animation import ImageAnimation

BASIC_ANIMATIONS = {
        Slide.ANIMATION_NAME : Slide,
        ImageAnimation.ANIMATION_NAME : ImageAnimation
}
