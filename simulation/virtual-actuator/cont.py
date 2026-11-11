"""Exact zero-order-hold discretisation of the unity-gain second-order lag."""
import numpy as np
from scipy.signal import cont2discrete


def Ad_Bd(wn, zeta, h):
    A = np.array([[0.0, 1.0], [-wn * wn, -2 * zeta * wn]])
    B = np.array([[0.0], [wn * wn]])
    Ad, Bd, *_ = cont2discrete((A, B, np.eye(2), np.zeros((2, 1))), h, method='zoh')
    return Ad, Bd[:, 0]
