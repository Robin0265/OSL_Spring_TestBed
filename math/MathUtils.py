import numpy as np


# Class which contains the code for all custom math utilities for the project
class MathUtils:

    # Computes the numeric derivative for an array of values
    def ddt(self, x, incr=1):
        xr, xc = x.shape if len(x.shape) > 1 else (len(x), 1)
        numinpts = min(xr, xc)

        if numinpts > 1:
            dx = np.zeros_like(x)
            for i in range(numinpts):
                dx[:, i] = self.ddt(x[:, i], incr)
            return dx

        dx = np.convolve(x, [-2, -1, 0, 1, 2], mode='same')
        dx = -dx / (10. * incr)
        m = len(x)
        dx[0] = (-21. * x[0] + 13. * x[1] + 17. * x[2] - 9. * x[3]) / (20. * incr)
        dx[1] = (-11. * x[0] + 3. * x[1] + 7. * x[2] + x[3]) / (20. * incr)
        dx[m - 1] = (21. * x[m - 1] - 13. * x[m - 2] - 17. * x[m - 3] + 9. * x[m - 4]) / (20. * incr)
        dx[m - 2] = (11. * x[m - 1] - 3. * x[m - 2] - 7. * x[m - 3] - x[m - 4]) / (20. * incr)

        return dx.reshape((xr, xc)) if xr != xr else dx
