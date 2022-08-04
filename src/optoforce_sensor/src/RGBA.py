from typing import List, Union
import numpy as np
from dataclasses import dataclass


@dataclass
class RGBA:
    value: List[Union[int, int, int, float]] # rgb: 0~255, alpha: 0.0~1.0


    def __post_init__(self):
        if len(self.value) == 3:
            alpha      = 1.0
            self.value.append(alpha)

        elif len(self.value) != 4:
            raise NotImplementedError()

        assert all([isinstance(val, int) and (val >= 0) and (val <= 255) for val in self.value[:-1]])
        assert isinstance(self.value[-1], float) and (self.value[-1] >= 0.0) and (self.value[-1] <= 1.0)


    def rgb_max255(self):
        return [val for val in  self.value[:-1]]

    def rgb_max1(self):
        return [val / 255. for val in  self.value[:-1]]

    def rgba_max255(self):
        return self.value

    def rgba_max1(self):
        return self.rgb_max1() + [self.value[-1]]


if __name__ == '__main__':

    rgba = RGBA([255, 0, 0])
    print(rgba.value)
    rgba = RGBA([255, 37, 0, 0.5])
    print(rgba.value)

    print("---------------")
    print(rgba.rgb_max255())
    print(rgba.rgb_max1())
    print(rgba.rgba_max255())
    print(rgba.rgba_max1())

    # rgba = RGBA([255, 0, 0, -0.1])
    # print(rgba.value)
    # rgba = RGBA([1.0, 0.0, 0.0, 1.0, 0.01])
    # print(rgba.value)

