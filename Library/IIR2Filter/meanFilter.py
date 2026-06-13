import numpy as np


class MeanFilter(object):
    def __init__(self, pipe_width):
        self.data_pipe = np.zeros(pipe_width)
        self.frame_id = 0
        self.early_flag = 1
        self.pipe_width=pipe_width

    def filter(self,data_new):
        self.data_pipe[self.frame_id] = data_new
        if self.frame_id == (self.pipe_width - 1):
            self.early_flag = 0
        if self.early_flag == 1:
            data_f = np.mean(self.data_pipe[0:self.frame_id + 1])
        else:
            data_f = np.mean(self.data_pipe)
        self.frame_id = (self.frame_id + 1) % self.pipe_width
        return data_f
