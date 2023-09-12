from Factories.ToolsFactory.GeneralTools import LowPassLiveFilter
import plotly.express as px
import numpy as np
import pandas as pd
if __name__ == "__main__":
    w = 1
    dt = 0.001
    t = np.arange(0, 1, step=dt)
    data = np.sin(2*np.pi*w * t)
    df = pd.DataFrame({'data': data}, index=t)
    fig = px.line(df)
    fig.show()
    lp_filter = LowPassLiveFilter([5], fs=1/dt, signals_num=1, filter_order=1)
    data_filtered = []
    for sample in data:
        data_filtered.append(lp_filter([sample])[0])
    df = pd.DataFrame({'data': data_filtered}, index=t)
    fig = px.line(df)
    fig.show()