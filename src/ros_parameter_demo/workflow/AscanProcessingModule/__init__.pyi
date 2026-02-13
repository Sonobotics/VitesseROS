import numpy.typing
import numpy as np


def singularThicknessCalculation(ascan: numpy.typing.NDArray[np.floating], samplingFrequency: int = 50 * (10 ** 6), lowboundTime: float = 6,
                                 minimumThickness: float = 5, waveVelocity: float = 3260, numCycles: int = 2, signalFrequency: int = int(3.6 * (10 ** 6)),
                                 thresholdSNR: float = 5, noiseWidth: int = 28, calibrationIndex: int = 42,
                                 temperature: float = 40, firstPeak: bool = False, multiEcho: bool = False, zeroCrossing: bool = False, temperatureCorrected: bool = False) -> tuple[numpy.typing.NDArray[np.floating], numpy.typing.NDArray[np.floating], float, numpy.typing.NDArray[np.floating], float]: ...
