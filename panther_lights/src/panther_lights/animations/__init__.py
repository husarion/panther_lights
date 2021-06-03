from .animation import Animation

from .slide import Slide
from .double_slide import DoubleSlide

BASIC_ANIMATIONS = {
        Slide.ANIMATION_NAME : Slide,
        DoubleSlide.ANIMATION_NAME : DoubleSlide
}
