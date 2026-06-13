
class RiseDetect:
    def __init__(self, ):
        self.flag = False
        self.flag_old = False
        self.force_enable_flag = False

    def step(self, input):
        self.flag_old = self.flag
        self.flag = input

        if self.force_enable_flag:
            self.force_enable_flag = False
            return True
        else:
            if self.flag and not self.flag_old:
                return True
            else:
                return False

    def force_enable(self):
        self.force_enable_flag = True