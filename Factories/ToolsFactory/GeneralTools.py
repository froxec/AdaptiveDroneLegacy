import numpy as np
import plotly.graph_objects as go
def manhattan_distance(a, b):
    return np.abs(a - b).sum()

def plot_signal(signals):
    """
    Function for plotting signals.

    :param signals: dict with keys ('x' - signal domain, 'y' - signal values, 'name' - signal name)
    """
    fig = go.Figure()
    for signal in signals:
        fig.add_trace(go.Scatter(x=signal['x'], y=signal['y'], name=signal['name']))
    fig.show()