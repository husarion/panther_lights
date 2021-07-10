import abc

class Controller(abc.ABC):

    def __init__(self, num_led, panel_count, brightness):
        '''LED controller interface'''
        raise NotImplementedError


    @abc.abstractmethod
    def set_panel(self, panel_num, panel_frame, brightness):
        '''sets panel_frame on panel_num led panel'''
        raise NotImplementedError


    @abc.abstractmethod
    def set_panel_state(self, panel_num, state):
        '''locks or unlocks given panel'''
        raise NotImplementedError


    @abc.abstractmethod
    def set_brightness(self, bright):
        '''sets panel brightness'''
        raise NotImplementedError

    @abc.abstractmethod
    def clear_panel(self, panel_num):
        '''clears panel'''
        raise NotImplementedError