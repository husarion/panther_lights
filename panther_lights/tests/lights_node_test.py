import unittest


class TestStringMethods(unittest.TestCase):

    def yaml_events_single_both(self):
        conf_yaml = yaml.load(open('config/events_single_both','r'), Loader=yaml.Loader)
        ex = Executor(conf_yaml['event'], 40, 0.01, 255)

        # Check type
        self.assertTrue(isinstance(ex._front_animation, ImageAnimation))
        self.assertTrue(isinstance(ex._tail_animation, ImageAnimation))

        # Animation values
        self.assertTrue(ex._front_animation._num_led == 40)
        self.assertTrue(ex._front_animation._time_step == 0.01)
        self.assertTrue(ex._front_animation._brightness == 255)

        self.assertTrue(ex._front_animation._duration == 1)
        self.assertTrue(ex._front_animation._loops == 1)
        self.assertTrue(ex._front_animation._keep_state == False)



if __name__ == '__main__':
    unittest.main()
