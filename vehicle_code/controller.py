import numpy as np
import scipy.io as spio
from scipy.interpolate import interp1d


class ControllerGainScheduler:
    # ControllerGainScheduler Storage class for controller gain scheduled gains

    def __init__(self, data):
        # GainScheduler constructor initializes gain scheduler
        #     Input: data - Structure loaded from MAT file

        self.vx_max = data['controller'][0, 0].lateral[0, 0].vx_max[0, 0]
        self.vx_min = data['controller'][0, 0].lateral[0, 0].vx_min[0, 0]

        vx_list = data['controller'][0, 0].lateral[0, 0].vx[:, 0]
        K = np.transpose(data['controller'][0, 0].lateral[0, 0].K, [1, 2, 0])
        self.K = interp1d(vx_list, K)

    def query(self, vx):
        # query Query controller gains
        #     Input: vx - Current longitudinal velocity [m/s]
        #     Output: K - Controller gain

        vx = max(min(vx, self.vx_max), self.vx_min)
        return np.matrix(self.K(vx))


class ObserverGainScheduler:
    # ControllerGainScheduler Storage class for observer gain scheduled gains

    def __init__(self, data):
        # GainScheduler constructor initializes gain scheduler
        #     Input: data - Structure loaded from MAT file

        self.vx_max = data['observer'][0, 0].vx_max[0, 0]
        self.vx_min = data['observer'][0, 0].vx_min[0, 0]

        vx_list = data['observer'][0, 0].vx[:, 0]
        F = np.transpose(data['observer'][0, 0].F, [1, 2, 0])
        G = np.transpose(np.reshape(data['observer'][0, 0].G,
                                    (vx_list.shape[0], -1, 1)), [1, 2, 0])
        Gr = np.transpose(np.reshape(data['observer'][0, 0].Gr,
                                     (vx_list.shape[0], -1, 1)), [1, 2, 0])
        L = np.transpose(data['observer'][0, 0].L, [1, 2, 0])
        self.F = interp1d(vx_list, F)
        self.G = interp1d(vx_list, G)
        self.Gr = interp1d(vx_list, Gr)
        self.L = interp1d(vx_list, L)

    def query(self, vx):
        # query Query controller gains
        #     Input: vx - Current longitudinal velocity [m/s]
        #     Output: F - State transition matrix
        #             G - Control input matrix
        #             Gr - Reference input matrix
        #             L - Observer gain matrix

        vx = max(min(vx, self.vx_max), self.vx_min)
        return np.matrix(self.F(vx)), np.matrix(self.G(vx)), np.matrix(self.Gr(vx)), \
            np.matrix(self.L(vx))


class Controller:
    # Controller main controller class with measurement, prediction and feedback
    #
    # Usage: controller = Controller()
    #        controller.predict(u, r, vx)
    #        controller.measure(y, vx)
    #        u = controller.feedback(r, vx)

    def __init__(self):
        # Controller constructor initializes gain schedulers and vehicle state

        data = spio.loadmat('controller_gains.mat', struct_as_record=False)

        self.control = ControllerGainScheduler(data)
        self.observer = ObserverGainScheduler(data)
        self.x = np.matrix(np.zeros((4, 1)))

    def predict(self, u, r, vx):
        # measure Execute the observer based on GPS data
        #     Input: u - Measured control input [steering]
        #            r - Reference vector [path curvature]
        #            vx - Current longitudinal velocity [m/s]

        F, G, Gr, L = self.observer.query(vx)
        self.x = F * self.x + G * u + Gr * r

    def measure(self, y, vx):
        # measure Execute the observer based on GPS data
        #     Input: y - Output vector [crosstrack error, heading error]
        #            vx - Current longitudinal velocity [m/s]

        _, _, _, L = self.observer.query(vx)
        Lc = np.eye(4) - L * np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.x = Lc * self.x + L * y

    def feedback(self, r, vx):
        # measure Execute the controller
        #     Input: r - Reference vector [path curvature]
        #            vx - Current longitudinal velocity [m/s]

        K = self.control.query(vx)
        v = np.vstack([self.x, r])
        return - K * v

    def get_x(self):
        # get_x Getter for vehicle state
        return self.x
